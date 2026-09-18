//! Multi-topic worker that forwards application telemetry to Grafana Live.

use std::{
    collections::BTreeMap,
    fs, io,
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use crate::{
    devices::lidar::{LidarPointCloudMessage, LidarRawMessage},
    models::{
        telemetry::{
            PowerThermalTelemetry, StorageHealthTelemetry, SystemHealthTelemetry,
            WorkerHealthTelemetry,
        },
        topics,
    },
    platform::{
        message_bus::{MessageBus, WorkerTopicInput, WorkerTopicInputs},
        pubsub::ReceiveStatus,
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
    },
    transport::http::HttpClient,
};

pub const GRAFANA_TOKEN_ENVIRONMENT_VARIABLE: &str = "VISTA_GRAFANA_TOKEN";
pub const GRAFANA_SECRET_FILE_NAME: &str = "GrafanaSecret.txt";

#[derive(Debug, Clone)]
pub struct GrafanaBridgeConfig {
    pub enabled: bool,
    pub host: String,
    pub port: u16,
    pub namespace: String,
    pub publish_interval: Duration,
    pub retry_interval: Duration,
    pub request_timeout: Duration,
    pub offline_timeout: Duration,
}

impl Default for GrafanaBridgeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            host: "127.0.0.1".to_owned(),
            port: 3000,
            namespace: "vista".to_owned(),
            publish_interval: Duration::from_secs(1),
            retry_interval: Duration::from_secs(5),
            request_timeout: Duration::from_secs(2),
            offline_timeout: Duration::from_secs(5),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct PointCloudTelemetry {
    pub lidar_id: String,
    pub timestamp_ns: u64,
    pub input_point_count: u64,
    pub point_count: u64,
    pub dropped_messages: u64,
    pub frames_per_second: f64,
    pub processing_ms: f64,
    pub online: bool,
    pub bounds: Option<[f32; 6]>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GrafanaBridgeReport {
    pub received_raw_messages: u64,
    pub received_decoded_messages: u64,
    pub received_processed_messages: u64,
    pub published_measurements: u64,
    pub failed_publish_attempts: u64,
    pub dropped_input_messages: u64,
}

pub fn validate_grafana_bridge_config(config: &GrafanaBridgeConfig) -> io::Result<()> {
    if config.host.is_empty() || config.port == 0 {
        return Err(invalid_input("Grafana host and port must be configured"));
    }
    if config.namespace.is_empty()
        || config.namespace.len() > 64
        || !config
            .namespace
            .bytes()
            .all(|value| value.is_ascii_alphanumeric() || matches!(value, b'-' | b'_'))
    {
        return Err(invalid_input(
            "Grafana namespace may contain only letters, digits, '-' or '_'",
        ));
    }
    if config.publish_interval.is_zero()
        || config.retry_interval.is_zero()
        || config.request_timeout.is_zero()
        || config.offline_timeout.is_zero()
    {
        return Err(invalid_input("Grafana timing values must be positive"));
    }
    Ok(())
}

pub fn make_grafana_bearer_authorization(token: &str) -> io::Result<String> {
    if !token.starts_with("glsa_") {
        return Err(invalid_input(
            "Grafana service-account token must begin with 'glsa_'",
        ));
    }
    if token.chars().any(char::is_whitespace) {
        return Err(invalid_input(
            "Grafana bearer token cannot contain whitespace",
        ));
    }
    Ok(format!("Bearer {token}"))
}

pub fn format_pointcloud_measurement(value: &PointCloudTelemetry) -> String {
    let bounds = value.bounds.unwrap_or([0.0; 6]);
    format!(
        "pointcloud,lidar_id={} online={}i,input_point_count={}i,point_count={}i,has_points={}i,fps={},processing_ms={},dropped_messages={}i,min_x_m={},max_x_m={},min_y_m={},max_y_m={},min_z_m={},max_z_m={} {}",
        escape_tag(&value.lidar_id),
        u8::from(value.online),
        value.input_point_count,
        value.point_count,
        u8::from(value.bounds.is_some()),
        value.frames_per_second,
        value.processing_ms,
        value.dropped_messages,
        bounds[0],
        bounds[1],
        bounds[2],
        bounds[3],
        bounds[4],
        bounds[5],
        value.timestamp_ns
    )
}

pub fn spawn_grafana_bridge<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    config: GrafanaBridgeConfig,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<GrafanaBridgeReport, String>) + Send + 'static,
{
    validate_grafana_bridge_config(&config)?;
    let mut inputs = WorkerTopicInputs::new(thread_config.name.clone())?;
    let raw = inputs.subscribe::<LidarRawMessage>(&bus, topics::LIDAR_RAW)?;
    let decoded = inputs.subscribe::<LidarPointCloudMessage>(&bus, topics::POINTCLOUD_DECODED)?;
    let processed =
        inputs.subscribe::<LidarPointCloudMessage>(&bus, topics::POINTCLOUD_PROCESSED)?;
    let system = inputs.subscribe::<SystemHealthTelemetry>(&bus, topics::SYSTEM_HEALTH)?;
    let worker = inputs.subscribe::<WorkerHealthTelemetry>(&bus, topics::WORKER_HEALTH)?;
    let storage = inputs.subscribe::<StorageHealthTelemetry>(&bus, topics::STORAGE_HEALTH)?;
    let power = inputs.subscribe::<PowerThermalTelemetry>(&bus, topics::POWER_THERMAL)?;

    spawn_worker(thread_config, move || {
        let result = run_grafana_bridge(
            inputs, raw, decoded, processed, system, worker, storage, power, config,
        )
        .map_err(|error| error.to_string());
        if result.is_err() {
            stop.request_stop();
        }
        on_complete(result);
    })
}

#[allow(clippy::too_many_arguments)]
fn run_grafana_bridge(
    inputs: WorkerTopicInputs,
    raw: WorkerTopicInput<LidarRawMessage>,
    decoded: WorkerTopicInput<LidarPointCloudMessage>,
    processed: WorkerTopicInput<LidarPointCloudMessage>,
    system: WorkerTopicInput<SystemHealthTelemetry>,
    worker: WorkerTopicInput<WorkerHealthTelemetry>,
    storage: WorkerTopicInput<StorageHealthTelemetry>,
    power: WorkerTopicInput<PowerThermalTelemetry>,
    config: GrafanaBridgeConfig,
) -> io::Result<GrafanaBridgeReport> {
    let mut publisher = GrafanaPublisher::new(config.clone())?;
    let mut report = GrafanaBridgeReport::default();
    let mut closed = [false; 7];
    let mut last_cloud = Instant::now()
        .checked_sub(config.offline_timeout)
        .unwrap_or_else(Instant::now);
    let mut next_publish = Instant::now();
    let mut lidar_id = "selected-lidar".to_owned();
    let mut input_count = 0_u64;
    let mut pending_cloud: Option<Arc<LidarPointCloudMessage>> = None;
    let mut system_value = None;
    let mut worker_values = BTreeMap::<String, WorkerHealthTelemetry>::new();
    let mut storage_value = None;
    let mut power_value = None;
    let mut raw_previous = 0;
    let mut decoded_previous = 0;
    let mut processed_previous = 0;
    let mut raw_fps = 0.0;
    let mut decoded_fps = 0.0;
    let mut processed_fps = 0.0;

    loop {
        let timeout = next_publish.saturating_duration_since(Instant::now());
        let ready = inputs.wait_timeout(timeout)?;
        drain(&raw, ready, &mut closed[0], |message| {
            report.received_raw_messages += 1;
            lidar_id = message.lidar_id.clone();
            raw_fps = update_rate(message.received_timestamp_ns, &mut raw_previous, raw_fps);
        })?;
        drain(&decoded, ready, &mut closed[1], |message| {
            report.received_decoded_messages += 1;
            input_count = message.payload.points.len() as u64;
            decoded_fps = update_rate(
                message.received_timestamp_ns,
                &mut decoded_previous,
                decoded_fps,
            );
        })?;
        drain(&processed, ready, &mut closed[2], |message| {
            report.received_processed_messages += 1;
            processed_fps = update_rate(
                message.received_timestamp_ns,
                &mut processed_previous,
                processed_fps,
            );
            last_cloud = Instant::now();
            pending_cloud = Some(message);
        })?;
        drain(&system, ready, &mut closed[3], |message| {
            system_value = Some((*message).clone());
        })?;
        drain(&worker, ready, &mut closed[4], |message| {
            worker_values.insert(message.worker_name.clone(), (*message).clone());
        })?;
        drain(&storage, ready, &mut closed[5], |message| {
            storage_value = Some((*message).clone());
        })?;
        drain(&power, ready, &mut closed[6], |message| {
            power_value = Some((*message).clone());
        })?;

        if Instant::now() >= next_publish {
            let online = last_cloud.elapsed() < config.offline_timeout;
            let drops =
                raw.dropped_messages() + decoded.dropped_messages() + processed.dropped_messages();
            let mut lines = Vec::new();
            if let Some(cloud) = pending_cloud.take() {
                lines.push(format_pointcloud_measurement(&summarize_point_cloud(
                    &cloud,
                    input_count,
                    drops,
                    processed_fps,
                )));
            } else if !online {
                lines.push(format_pointcloud_measurement(&PointCloudTelemetry {
                    lidar_id: lidar_id.clone(),
                    timestamp_ns: system_timestamp_ns(),
                    ..PointCloudTelemetry::default()
                }));
            }
            lines.push(format!(
                "pipeline_health online={}i,raw_fps={},decoded_fps={},processed_fps={},end_to_end_latency_ms=0,raw_queue_fill_percent=0,decoded_queue_fill_percent=0,processed_queue_fill_percent=0,dropped_messages={}i,last_message_age_ms={} {}",
                u8::from(online),
                if online { raw_fps } else { 0.0 },
                if online { decoded_fps } else { 0.0 },
                if online { processed_fps } else { 0.0 },
                drops,
                last_cloud.elapsed().as_secs_f64() * 1000.0,
                system_timestamp_ns()
            ));
            if let Some(value) = &system_value {
                lines.push(format_system_health(value));
            }
            lines.extend(worker_values.values().map(format_worker_health));
            if let Some(value) = &storage_value {
                lines.push(format_storage_health(value));
            }
            if let Some(value) = &power_value {
                lines.push(format_power_thermal(value));
            }
            publisher.publish(lines, inputs.topic_count(), &mut report);
            next_publish = Instant::now() + config.publish_interval;
        }
        if closed.iter().all(|value| *value) {
            break;
        }
    }
    report.dropped_input_messages = raw.dropped_messages()
        + decoded.dropped_messages()
        + processed.dropped_messages()
        + system.dropped_messages()
        + worker.dropped_messages()
        + storage.dropped_messages()
        + power.dropped_messages();
    Ok(report)
}

fn drain<T, F>(
    input: &WorkerTopicInput<T>,
    ready: u64,
    closed: &mut bool,
    mut handler: F,
) -> io::Result<()>
where
    T: Send + Sync + 'static,
    F: FnMut(Arc<T>),
{
    if *closed || !input.is_ready(ready) {
        return Ok(());
    }
    loop {
        match input.try_receive()? {
            ReceiveStatus::Message(message) => handler(message),
            ReceiveStatus::Timeout => return Ok(()),
            ReceiveStatus::Closed => {
                *closed = true;
                return Ok(());
            }
        }
    }
}

struct GrafanaPublisher {
    config: GrafanaBridgeConfig,
    client: HttpClient,
    path: String,
    headers: Vec<(String, String)>,
    next_retry: Option<Instant>,
    last_success: Option<Instant>,
    last_http_status: u16,
    retrying: bool,
}

impl GrafanaPublisher {
    fn new(config: GrafanaBridgeConfig) -> io::Result<Self> {
        let mut headers = Vec::new();
        match load_grafana_authorization() {
            Ok(Some((authorization, source))) => {
                headers.push(("Authorization".to_owned(), authorization));
                println!("Grafana authentication: Bearer token loaded from {source}");
            }
            Ok(None) => eprintln!(
                "Grafana authentication warning: neither {GRAFANA_SECRET_FILE_NAME} nor {GRAFANA_TOKEN_ENVIRONMENT_VARIABLE} contains a token"
            ),
            Err(error) => eprintln!("Grafana authentication warning: {error}"),
        }
        Ok(Self {
            client: HttpClient::new(&config.host, config.port, config.request_timeout)?,
            path: format!("/api/live/push/{}", config.namespace),
            config,
            headers,
            next_retry: None,
            last_success: None,
            last_http_status: 0,
            retrying: false,
        })
    }

    fn publish(
        &mut self,
        mut lines: Vec<String>,
        topic_count: usize,
        report: &mut GrafanaBridgeReport,
    ) {
        let now = Instant::now();
        if self.next_retry.is_some_and(|deadline| now < deadline) {
            return;
        }
        lines.push(self.health_line(topic_count, report));
        let body = lines.join("\n") + "\n";
        match self.client.post(
            &self.path,
            "text/plain; charset=utf-8",
            &body,
            &self.headers,
        ) {
            Ok(response) if (200..300).contains(&response.status_code) => {
                report.published_measurements += lines.len() as u64;
                self.last_http_status = response.status_code;
                self.last_success = Some(now);
                self.next_retry = None;
                if self.retrying {
                    println!(
                        "Grafana connection restored: http://{}:{}{}",
                        self.config.host, self.config.port, self.path
                    );
                }
                self.retrying = false;
            }
            result => {
                report.failed_publish_attempts += 1;
                let message = match result {
                    Ok(response) => {
                        self.last_http_status = response.status_code;
                        format!(
                            "Grafana returned HTTP {} {}",
                            response.status_code, response.reason
                        )
                    }
                    Err(error) => error.to_string(),
                };
                self.next_retry = Some(now + self.config.retry_interval);
                if !self.retrying {
                    eprintln!(
                        "Grafana publish failed: {message}. Retrying in {:.3} second(s).",
                        self.config.retry_interval.as_secs_f64()
                    );
                }
                self.retrying = true;
            }
        }
    }

    fn health_line(&self, topic_count: usize, report: &GrafanaBridgeReport) -> String {
        let success_age_ms = self
            .last_success
            .map_or(-1.0, |value| value.elapsed().as_secs_f64() * 1000.0);
        format!(
            "grafana_bridge_health published_measurements={}i,failed_publish_attempts={}i,last_http_status={}i,last_success_age_ms={},retrying={}i,subscribed_topics={}i {}",
            report.published_measurements,
            report.failed_publish_attempts,
            self.last_http_status,
            success_age_ms,
            u8::from(self.retrying),
            topic_count,
            system_timestamp_ns()
        )
    }
}

fn load_grafana_authorization() -> io::Result<Option<(String, &'static str)>> {
    if let Some(token) = read_grafana_token_file()? {
        return Ok(Some((
            make_grafana_bearer_authorization(&token)?,
            GRAFANA_SECRET_FILE_NAME,
        )));
    }
    if let Ok(token) = std::env::var(GRAFANA_TOKEN_ENVIRONMENT_VARIABLE) {
        if !token.trim().is_empty() {
            return Ok(Some((
                make_grafana_bearer_authorization(token.trim())?,
                GRAFANA_TOKEN_ENVIRONMENT_VARIABLE,
            )));
        }
    }
    Ok(None)
}

fn read_grafana_token_file() -> io::Result<Option<String>> {
    let Ok(contents) = fs::read_to_string(GRAFANA_SECRET_FILE_NAME) else {
        return Ok(None);
    };
    let mut token = None;
    for (index, line) in contents.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (key, value) = line.split_once(':').ok_or_else(|| {
            invalid_input(format!(
                "{GRAFANA_SECRET_FILE_NAME} line {} must use 'GrafanaToken: <glsa_...>'",
                index + 1
            ))
        })?;
        if key.trim() != "GrafanaToken" || token.is_some() {
            return Err(invalid_input(format!(
                "{GRAFANA_SECRET_FILE_NAME} accepts one GrafanaToken entry"
            )));
        }
        if !value.trim().is_empty() {
            token = Some(value.trim().to_owned());
        }
    }
    Ok(token)
}

fn summarize_point_cloud(
    message: &LidarPointCloudMessage,
    input_point_count: u64,
    dropped_messages: u64,
    frames_per_second: f64,
) -> PointCloudTelemetry {
    let timestamp_ns = system_timestamp_ns();
    let bounds = message.payload.points.first().map(|first| {
        message.payload.points.iter().skip(1).fold(
            [first.x, first.x, first.y, first.y, first.z, first.z],
            |mut value, point| {
                value[0] = value[0].min(point.x);
                value[1] = value[1].max(point.x);
                value[2] = value[2].min(point.y);
                value[3] = value[3].max(point.y);
                value[4] = value[4].min(point.z);
                value[5] = value[5].max(point.z);
                value
            },
        )
    });
    PointCloudTelemetry {
        lidar_id: message.lidar_id.clone(),
        timestamp_ns,
        input_point_count,
        point_count: message.payload.points.len() as u64,
        dropped_messages,
        frames_per_second,
        processing_ms: timestamp_ns.saturating_sub(message.received_timestamp_ns) as f64
            / 1_000_000.0,
        online: true,
        bounds,
    }
}

fn update_rate(timestamp: u64, previous: &mut u64, old_rate: f64) -> f64 {
    let result = if *previous != 0 && timestamp > *previous {
        1_000_000_000.0 / (timestamp - *previous) as f64
    } else {
        old_rate
    };
    *previous = timestamp;
    result
}

fn format_system_health(value: &SystemHealthTelemetry) -> String {
    format!(
        "system_health online=1i,uptime_seconds={},cpu_percent={},memory_mb={},memory_percent={},worker_count={}i,failed_workers={}i {}",
        value.uptime_seconds,
        value.cpu_percent,
        value.memory_mb,
        value.memory_percent,
        value.worker_count,
        value.failed_workers,
        value.timestamp_ns
    )
}

fn format_worker_health(value: &WorkerHealthTelemetry) -> String {
    format!(
        "worker_health,worker={} running={}i,failed={}i,priority={}i,uptime_seconds={} {}",
        escape_tag(&value.worker_name),
        u8::from(value.running),
        u8::from(value.failed),
        value.priority,
        value.uptime_seconds,
        value.timestamp_ns
    )
}

fn format_storage_health(value: &StorageHealthTelemetry) -> String {
    format!(
        "storage_health disk_free_gb={},raw_bytes_written={}i,raw_messages_written={}i,pcd_points_written={}i,pcd_frames_written={}i,write_errors={}i {}",
        value.disk_free_gb,
        value.raw_bytes_written,
        value.raw_messages_written,
        value.pcd_points_written,
        value.pcd_frames_written,
        value.write_errors,
        value.timestamp_ns
    )
}

fn format_power_thermal(value: &PowerThermalTelemetry) -> String {
    format!(
        "power_thermal available={}i,cpu_temperature_c={},gpu_temperature_c={},board_power_w={},fan_rpm={} {}",
        u8::from(value.available),
        value.cpu_temperature_c,
        value.gpu_temperature_c,
        value.board_power_w,
        value.fan_rpm,
        value.timestamp_ns
    )
}

fn escape_tag(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for character in value.chars() {
        if matches!(character, ',' | '=' | ' ') {
            escaped.push('\\');
        }
        escaped.push(character);
    }
    escaped
}

fn system_timestamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(u128::from(u64::MAX)) as u64
}

fn invalid_input(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

#[cfg(test)]
#[path = "../../unittest/application_test/grafana_bridge_test.rs"]
mod tests;
