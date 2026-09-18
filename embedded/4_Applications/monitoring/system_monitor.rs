//! Independent worker that publishes runtime, storage, and worker telemetry.

use std::{
    io,
    path::PathBuf,
    sync::Arc,
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

#[cfg(target_os = "linux")]
use std::fs;

use crate::{
    models::{
        telemetry::{
            PowerThermalTelemetry, StorageHealthTelemetry, SystemHealthTelemetry,
            WorkerHealthTelemetry,
        },
        topics,
    },
    platform::{
        disk_free_gb,
        message_bus::MessageBus,
        process_memory_mb,
        runtime_metrics::{storage_metrics_snapshot, worker_runtime_snapshot},
        system_memory_percent,
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
        CpuSampler,
    },
};

#[derive(Debug, Clone)]
pub struct SystemMonitorConfig {
    pub sample_interval: Duration,
    pub data_root: PathBuf,
}

impl Default for SystemMonitorConfig {
    fn default() -> Self {
        Self {
            sample_interval: Duration::from_secs(2),
            data_root: PathBuf::from("../data"),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SystemMonitorReport {
    pub sample_count: u64,
}

pub fn spawn_system_monitor<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    config: SystemMonitorConfig,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<SystemMonitorReport, String>) + Send + 'static,
{
    if config.sample_interval.is_zero() || config.data_root.as_os_str().is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid system-monitor configuration",
        ));
    }
    let system_publisher = bus.publisher::<SystemHealthTelemetry>(topics::SYSTEM_HEALTH)?;
    let worker_publisher = bus.publisher::<WorkerHealthTelemetry>(topics::WORKER_HEALTH)?;
    let storage_publisher = bus.publisher::<StorageHealthTelemetry>(topics::STORAGE_HEALTH)?;
    let power_publisher = bus.publisher::<PowerThermalTelemetry>(topics::POWER_THERMAL)?;

    spawn_worker(thread_config, move || {
        let result = run_system_monitor(
            system_publisher,
            worker_publisher,
            storage_publisher,
            power_publisher,
            config,
            &stop,
        )
        .map_err(|error| error.to_string());
        if result.is_err() {
            stop.request_stop();
        }
        on_complete(result);
    })
}

fn run_system_monitor(
    system_publisher: crate::platform::pubsub::TopicPublisher<SystemHealthTelemetry>,
    worker_publisher: crate::platform::pubsub::TopicPublisher<WorkerHealthTelemetry>,
    storage_publisher: crate::platform::pubsub::TopicPublisher<StorageHealthTelemetry>,
    power_publisher: crate::platform::pubsub::TopicPublisher<PowerThermalTelemetry>,
    config: SystemMonitorConfig,
    stop: &StopToken,
) -> io::Result<SystemMonitorReport> {
    let started = Instant::now();
    let mut next_sample = started;
    let mut report = SystemMonitorReport::default();
    let mut cpu = CpuSampler::default();

    while !stop.is_stop_requested() {
        let timestamp_ns = system_timestamp_ns();
        let workers = worker_runtime_snapshot();
        for worker in &workers {
            worker_publisher.publish(WorkerHealthTelemetry {
                timestamp_ns,
                worker_name: worker.name.clone(),
                priority: worker.priority,
                running: worker.running,
                failed: worker.failed,
                uptime_seconds: worker.uptime_seconds,
            })?;
        }

        system_publisher.publish(SystemHealthTelemetry {
            timestamp_ns,
            uptime_seconds: started.elapsed().as_secs_f64(),
            cpu_percent: cpu.sample(),
            memory_mb: process_memory_mb(),
            memory_percent: system_memory_percent(),
            worker_count: workers.iter().filter(|worker| worker.running).count() as u64,
            failed_workers: workers.iter().filter(|worker| worker.failed).count() as u64,
        })?;

        let storage = storage_metrics_snapshot();
        storage_publisher.publish(StorageHealthTelemetry {
            timestamp_ns,
            disk_free_gb: disk_free_gb(&config.data_root),
            raw_bytes_written: storage.raw_bytes_written,
            raw_messages_written: storage.raw_messages_written,
            pcd_points_written: storage.pcd_points_written,
            pcd_frames_written: storage.pcd_frames_written,
            write_errors: storage.write_errors,
        })?;
        power_publisher.publish(power_thermal_sample(timestamp_ns))?;
        report.sample_count += 1;

        next_sample += config.sample_interval;
        while !stop.is_stop_requested() && Instant::now() < next_sample {
            thread::sleep(
                next_sample
                    .saturating_duration_since(Instant::now())
                    .min(Duration::from_millis(50)),
            );
        }
        if Instant::now() > next_sample + config.sample_interval {
            next_sample = Instant::now();
        }
    }
    Ok(report)
}

#[cfg(target_os = "linux")]
fn power_thermal_sample(timestamp_ns: u64) -> PowerThermalTelemetry {
    let mut result = PowerThermalTelemetry {
        timestamp_ns,
        ..PowerThermalTelemetry::default()
    };
    if let Ok(entries) = fs::read_dir("/sys/class/thermal") {
        for entry in entries.flatten() {
            if !entry
                .file_name()
                .to_string_lossy()
                .starts_with("thermal_zone")
            {
                continue;
            }
            let Some(raw) = read_number(entry.path().join("temp")) else {
                continue;
            };
            let sensor_type = fs::read_to_string(entry.path().join("type")).unwrap_or_default();
            let temperature = if raw > 1_000.0 { raw / 1_000.0 } else { raw };
            if sensor_type.to_ascii_lowercase().contains("gpu") {
                result.gpu_temperature_c = result.gpu_temperature_c.max(temperature);
            } else {
                result.cpu_temperature_c = result.cpu_temperature_c.max(temperature);
            }
            result.available = true;
        }
    }
    if let Ok(hwmon_entries) = fs::read_dir("/sys/class/hwmon") {
        for directory in hwmon_entries.flatten() {
            let Ok(entries) = fs::read_dir(directory.path()) else {
                continue;
            };
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().into_owned();
                let Some(raw) = read_number(entry.path()) else {
                    continue;
                };
                if name.starts_with("power") && name.contains("_input") {
                    result.board_power_w += raw / 1_000_000.0;
                    result.available = true;
                } else if name.starts_with("fan") && name.contains("_input") {
                    result.fan_rpm = result.fan_rpm.max(raw);
                    result.available = true;
                }
            }
        }
    }
    result
}

#[cfg(target_os = "linux")]
fn read_number(path: PathBuf) -> Option<f64> {
    fs::read_to_string(path).ok()?.trim().parse().ok()
}

#[cfg(not(target_os = "linux"))]
fn power_thermal_sample(timestamp_ns: u64) -> PowerThermalTelemetry {
    PowerThermalTelemetry {
        timestamp_ns,
        ..PowerThermalTelemetry::default()
    }
}

fn system_timestamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(u128::from(u64::MAX)) as u64
}

#[cfg(test)]
#[path = "../../unittest/application_test/system_monitor_test.rs"]
mod tests;
