//! Loads device and runtime settings directly from DeviceConfig.txt.

use std::{
    collections::HashSet,
    fs,
    net::{IpAddr, Ipv4Addr},
    path::{Path, PathBuf},
    time::Duration,
};

use vista_edge::{
    application::{
        grafana_bridge::{validate_grafana_bridge_config, GrafanaBridgeConfig},
        pointcloud_processing::{GroundRemovalConfig, PreprocessingConfig},
        system_monitor::SystemMonitorConfig,
    },
    devices::lidar::{DepthStreamConfig, LidarConfig, LidarType},
    platform::{self, threading::ThreadConfig},
};

pub(crate) const DEFAULT_DEVICE_CONFIG_PATH: &str = "DeviceConfig.txt";
const DEFAULT_SENSOR_IP: IpAddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 3));
const DEFAULT_DEPTH_WIDTH: u32 = 640;
const DEFAULT_DEPTH_HEIGHT: u32 = 480;
const DEFAULT_DEPTH_FPS: u32 = 30;
const CONNECTION_TIMEOUT: Duration = Duration::from_secs(5);
const DEFAULT_RECONNECT_INTERVAL: Duration = Duration::from_secs(5);
const MAXIMUM_RECONNECT_INTERVAL_SECONDS: u64 = 86_400;
const MAXIMUM_GRAFANA_INTERVAL_MILLISECONDS: u64 = 3_600_000;

const LIDAR_READ_PRIORITY: u8 = 1;
const LIDAR_DECODE_PRIORITY: u8 = 2;
const RAW_LOGGER_PRIORITY: u8 = 2;
const PCD_LOGGER_PRIORITY: u8 = 2;
const PREPROCESSING_PRIORITY: u8 = 3;
const GRAFANA_BRIDGE_PRIORITY: u8 = 4;
const SYSTEM_MONITOR_PRIORITY: u8 = 4;

const RAW_TOPIC_CAPACITY: usize = 32;
const DECODED_POINTCLOUD_TOPIC_CAPACITY: usize = 8;
const PROCESSED_POINTCLOUD_TOPIC_CAPACITY: usize = 8;
const TELEMETRY_TOPIC_CAPACITY: usize = 32;

const MIN_DISTANCE_METERS: f32 = 0.1;
const MAX_DISTANCE_METERS: f32 = 200.0;
const VOXEL_SIZE_METERS: Option<f32> = Some(0.05);
const GROUND_HEIGHT_METERS: Option<f32> = None;
const GROUND_TOLERANCE_METERS: f32 = 0.15;

#[derive(Debug, Clone)]
pub(crate) struct WorkerConfig {
    pub enabled: bool,
    pub thread: ThreadConfig,
}

#[derive(Debug, Clone)]
pub(crate) struct ThreadSetConfig {
    pub lidar_read: WorkerConfig,
    pub lidar_decode: WorkerConfig,
    pub raw_logger: WorkerConfig,
    pub preprocessing: WorkerConfig,
    pub pcd_logger: WorkerConfig,
    pub grafana_bridge: WorkerConfig,
    pub system_monitor: WorkerConfig,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TopicQueueConfig {
    pub raw_capacity: usize,
    pub decoded_capacity: usize,
    pub processed_capacity: usize,
    pub telemetry_capacity: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RuntimeConfig {
    pub preprocessing: PreprocessingConfig,
    pub threads: ThreadSetConfig,
    pub queues: TopicQueueConfig,
}

/// Complete device selection and generated output paths for one process run.
#[derive(Debug, Clone)]
pub(crate) struct AppConfig {
    pub lidar_type: LidarType,
    pub sensor_ip: IpAddr,
    pub tcp_port: Option<u16>,
    pub usb_serial: Option<String>,
    pub depth_width: u32,
    pub depth_height: u32,
    pub depth_fps: u32,
    /// Enables the RAW logger worker and timestamped .bin output.
    pub raw_logging_enabled: bool,
    /// Enables the PCD logger worker and timestamped .pcd output.
    pub pointcloud_logging_enabled: bool,
    pub lidar_reconnect_interval: Duration,
    pub raw_path: Option<PathBuf>,
    pub pcd_path: Option<PathBuf>,
    pub grafana: GrafanaBridgeConfig,
    pub system_monitor: SystemMonitorConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            lidar_type: LidarType::QuanergyM8,
            sensor_ip: DEFAULT_SENSOR_IP,
            tcp_port: None,
            usb_serial: None,
            depth_width: DEFAULT_DEPTH_WIDTH,
            depth_height: DEFAULT_DEPTH_HEIGHT,
            depth_fps: DEFAULT_DEPTH_FPS,
            raw_logging_enabled: false,
            pointcloud_logging_enabled: false,
            lidar_reconnect_interval: DEFAULT_RECONNECT_INTERVAL,
            raw_path: None,
            pcd_path: None,
            grafana: GrafanaBridgeConfig::default(),
            system_monitor: SystemMonitorConfig::default(),
        }
    }
}

impl AppConfig {
    /// Reads settings and creates paths only for explicitly enabled logs.
    pub(crate) fn load() -> Result<Self, String> {
        let contents = fs::read_to_string(DEFAULT_DEVICE_CONFIG_PATH).map_err(|error| {
            format!("could not read device configuration '{DEFAULT_DEVICE_CONFIG_PATH}': {error}")
        })?;
        let mut config = parse_device_config(&contents)?;
        let data_root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .ok_or_else(|| "embedded directory has no repository parent".to_owned())?
            .join("data");
        config.system_monitor.data_root = data_root.clone();
        if config.raw_logging_enabled || config.pointcloud_logging_enabled {
            let log_stem = platform::current_log_timestamp().map_err(|error| error.to_string())?;
            if config.raw_logging_enabled {
                config.raw_path = Some(data_root.join("raw").join(format!("{log_stem}.bin")));
            }
            if config.pointcloud_logging_enabled {
                config.pcd_path = Some(data_root.join("processed").join(format!("{log_stem}.pcd")));
            }
        }
        Ok(config)
    }

    pub(crate) fn lidar_config(&self) -> LidarConfig {
        let mut config = LidarConfig::new(self.lidar_type, self.sensor_ip)
            .with_connection_timeout(CONNECTION_TIMEOUT);
        if let Some(port) = self.tcp_port {
            config = config.with_port(port);
        }
        if self.lidar_type == LidarType::RealSenseL515 {
            config = config.with_depth_stream(DepthStreamConfig::new(
                self.usb_serial.clone(),
                self.depth_width,
                self.depth_height,
                self.depth_fps,
                CONNECTION_TIMEOUT,
            ));
        }
        config
    }

    pub(crate) fn runtime_config(&self) -> Result<RuntimeConfig, String> {
        Ok(RuntimeConfig {
            preprocessing: preprocessing_config(),
            threads: ThreadSetConfig {
                lidar_read: worker_config("lidar-read", LIDAR_READ_PRIORITY)?,
                lidar_decode: worker_config("lidar-decode", LIDAR_DECODE_PRIORITY)?,
                raw_logger: worker_config_with_enabled(
                    "raw-logger",
                    RAW_LOGGER_PRIORITY,
                    self.raw_logging_enabled,
                )?,
                preprocessing: worker_config("pointcloud-preprocessing", PREPROCESSING_PRIORITY)?,
                pcd_logger: worker_config_with_enabled(
                    "pcd-logger",
                    PCD_LOGGER_PRIORITY,
                    self.pointcloud_logging_enabled,
                )?,
                grafana_bridge: worker_config("grafana-bridge", GRAFANA_BRIDGE_PRIORITY)?,
                system_monitor: worker_config("system-monitor", SYSTEM_MONITOR_PRIORITY)?,
            },
            queues: TopicQueueConfig {
                raw_capacity: RAW_TOPIC_CAPACITY,
                decoded_capacity: DECODED_POINTCLOUD_TOPIC_CAPACITY,
                processed_capacity: PROCESSED_POINTCLOUD_TOPIC_CAPACITY,
                telemetry_capacity: TELEMETRY_TOPIC_CAPACITY,
            },
        })
    }
}

fn worker_config(name: &str, priority: u8) -> Result<WorkerConfig, String> {
    worker_config_with_enabled(name, priority, true)
}

fn worker_config_with_enabled(
    name: &str,
    priority: u8,
    enabled: bool,
) -> Result<WorkerConfig, String> {
    Ok(WorkerConfig {
        enabled,
        thread: ThreadConfig::new(name, priority).map_err(|error| error.to_string())?,
    })
}

/// Validates dependencies between the worker switches chosen in main.
pub(crate) fn validate_worker_selection(
    app: &AppConfig,
    runtime: &RuntimeConfig,
) -> Result<(), String> {
    if runtime.threads.lidar_decode.enabled && !runtime.threads.lidar_read.enabled {
        return Err("lidar-decode requires the lidar-read worker".to_owned());
    }
    if runtime.threads.preprocessing.enabled && !runtime.threads.lidar_decode.enabled {
        return Err("pointcloud-preprocessing requires the lidar-decode worker".to_owned());
    }
    if app.raw_path.is_some()
        && runtime.threads.raw_logger.enabled
        && !runtime.threads.lidar_read.enabled
    {
        return Err("raw-logger requires the lidar-read worker".to_owned());
    }
    if app.pcd_path.is_some()
        && runtime.threads.pcd_logger.enabled
        && !runtime.threads.preprocessing.enabled
    {
        return Err("pcd-logger requires the pointcloud-preprocessing worker".to_owned());
    }
    Ok(())
}

fn without_device_note(value: &str) -> &str {
    let value = value.trim();
    if value.ends_with(')') {
        if let Some(index) = value.rfind('(') {
            return value[..index].trim();
        }
    }
    value
}

fn parse_positive_u64(value: &str, label: &str, maximum: u64) -> Result<u64, String> {
    let parsed = value
        .parse::<u64>()
        .map_err(|error| format!("invalid {label} '{value}': {error}"))?;
    if parsed == 0 || parsed > maximum {
        return Err(format!("{label} must be between 1 and {maximum}"));
    }
    Ok(parsed)
}

fn parse_positive_u32(value: &str, label: &str) -> Result<u32, String> {
    let parsed = parse_positive_u64(value, label, u64::from(u32::MAX))?;
    Ok(parsed as u32)
}

fn parse_boolean(value: &str, label: &str) -> Result<bool, String> {
    match value.to_ascii_lowercase().as_str() {
        "true" | "yes" | "on" | "1" => Ok(true),
        "false" | "no" | "off" | "0" => Ok(false),
        _ => Err(format!(
            "{label} must be true/false, yes/no, on/off, or 1/0"
        )),
    }
}

fn parse_zero_one_switch(value: &str, label: &str) -> Result<bool, String> {
    match value {
        "0" => Ok(false),
        "1" => Ok(true),
        _ => Err(format!("{label} must be 0 or 1")),
    }
}

/// Parses the same device file format used by the C++ application.
pub(crate) fn parse_device_config(contents: &str) -> Result<AppConfig, String> {
    let mut config = AppConfig::default();
    let mut keys = HashSet::new();
    let mut lidar_seen = false;
    let mut sensor_ip_seen = false;
    let mut tcp_port_seen = false;
    let mut usb_serial_seen = false;
    let mut depth_width_seen = false;
    let mut depth_height_seen = false;
    let mut depth_fps_seen = false;

    for (index, original_line) in contents.lines().enumerate() {
        let line = original_line
            .trim_start_matches('\u{feff}')
            .split('#')
            .next()
            .unwrap_or_default()
            .trim();
        if line.is_empty() {
            continue;
        }
        let (raw_key, raw_value) = line.split_once(':').ok_or_else(|| {
            format!(
                "invalid device configuration at line {}: expected 'Key: value'",
                index + 1
            )
        })?;
        let key = without_device_note(raw_key).to_ascii_lowercase();
        let value = without_device_note(raw_value);
        if !keys.insert(key.clone()) {
            return Err(format!(
                "duplicate device configuration key '{key}' at line {}",
                index + 1
            ));
        }

        match key.as_str() {
            "lidar" => {
                if value.is_empty() || value.eq_ignore_ascii_case("none") {
                    return Err(
                        "DeviceConfig.txt must select quanergy-m8 or realsense-l515".to_owned()
                    );
                }
                config.lidar_type = value.parse::<LidarType>()?;
                lidar_seen = true;
            }
            "radar" => {
                if !value.is_empty() && !value.eq_ignore_ascii_case("none") {
                    return Err("Radar is not implemented yet".to_owned());
                }
            }
            "sensorip" | "ip" => {
                config.sensor_ip = value
                    .parse::<IpAddr>()
                    .map_err(|error| format!("invalid sensor IP '{value}': {error}"))?;
                sensor_ip_seen = true;
            }
            "tcpport" | "port" => {
                config.tcp_port =
                    Some(parse_positive_u64(value, "TCP port", u64::from(u16::MAX))? as u16);
                tcp_port_seen = true;
            }
            "usbserial" | "comport" => {
                config.usb_serial = (!value.is_empty()).then(|| value.to_owned());
                usb_serial_seen = true;
            }
            "depthwidth" | "width" => {
                config.depth_width = parse_positive_u32(value, "depth width")?;
                depth_width_seen = true;
            }
            "depthheight" | "height" => {
                config.depth_height = parse_positive_u32(value, "depth height")?;
                depth_height_seen = true;
            }
            "depthfps" | "fps" => {
                config.depth_fps = parse_positive_u32(value, "depth frame rate")?;
                depth_fps_seen = true;
            }
            "reconnectintervalseconds" | "lidarreconnectintervalseconds" => {
                config.lidar_reconnect_interval = Duration::from_secs(parse_positive_u64(
                    value,
                    "LiDAR reconnect interval",
                    MAXIMUM_RECONNECT_INTERVAL_SECONDS,
                )?);
            }
            "rawloggingenabled" => {
                config.raw_logging_enabled =
                    parse_zero_one_switch(value, "RawLoggingEnabled(Lidar)")?;
            }
            "pointcloudloggingenabled" => {
                config.pointcloud_logging_enabled =
                    parse_zero_one_switch(value, "PointCloudLoggingEnabled(Lidar)")?;
            }
            "grafanaenabled" => {
                config.grafana.enabled = parse_boolean(value, "GrafanaEnabled")?;
            }
            "grafanahost" => {
                if value.is_empty() {
                    return Err("GrafanaHost cannot be empty".to_owned());
                }
                config.grafana.host = value.to_owned();
            }
            "grafanaport" => {
                config.grafana.port =
                    parse_positive_u64(value, "Grafana port", u64::from(u16::MAX))? as u16;
            }
            "grafananamespace" => {
                config.grafana.namespace = value.to_owned();
            }
            "grafanapublishintervalmilliseconds" => {
                config.grafana.publish_interval = Duration::from_millis(parse_positive_u64(
                    value,
                    "Grafana publish interval",
                    MAXIMUM_GRAFANA_INTERVAL_MILLISECONDS,
                )?);
            }
            "grafanaretryintervalseconds" => {
                config.grafana.retry_interval = Duration::from_secs(parse_positive_u64(
                    value,
                    "Grafana retry interval",
                    MAXIMUM_RECONNECT_INTERVAL_SECONDS,
                )?);
            }
            "systemmonitorintervalmilliseconds" => {
                config.system_monitor.sample_interval = Duration::from_millis(parse_positive_u64(
                    value,
                    "system monitor interval",
                    MAXIMUM_GRAFANA_INTERVAL_MILLISECONDS,
                )?);
            }
            _ => {
                return Err(format!("unknown device key '{key}' at line {}", index + 1));
            }
        }
    }

    if !lidar_seen {
        return Err("DeviceConfig.txt is missing a Lidar selection".to_owned());
    }
    if config.lidar_type == LidarType::QuanergyM8 && (!sensor_ip_seen || !tcp_port_seen) {
        return Err("Quanergy M8 requires SensorIP and TcpPort in DeviceConfig.txt".to_owned());
    }
    if config.lidar_type == LidarType::RealSenseL515
        && (!usb_serial_seen || !depth_width_seen || !depth_height_seen || !depth_fps_seen)
    {
        return Err(
            "RealSense L515 requires UsbSerial, DepthWidth, DepthHeight, and DepthFps in DeviceConfig.txt"
                .to_owned(),
        );
    }
    validate_grafana_bridge_config(&config.grafana).map_err(|error| error.to_string())?;
    Ok(config)
}

fn preprocessing_config() -> PreprocessingConfig {
    PreprocessingConfig {
        min_distance_m: MIN_DISTANCE_METERS,
        max_distance_m: MAX_DISTANCE_METERS,
        region_of_interest: None,
        voxel_size_m: VOXEL_SIZE_METERS,
        ground_removal: GROUND_HEIGHT_METERS.map(|ground_height_m| GroundRemovalConfig {
            ground_height_m,
            tolerance_m: GROUND_TOLERANCE_METERS,
        }),
    }
}

#[cfg(test)]
#[path = "unittest/config_test.rs"]
mod tests;
