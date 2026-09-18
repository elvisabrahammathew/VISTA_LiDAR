//! Sensor-neutral runtime telemetry published for monitoring applications.

#[derive(Debug, Clone, Default)]
pub struct SystemHealthTelemetry {
    pub timestamp_ns: u64,
    pub uptime_seconds: f64,
    pub cpu_percent: f64,
    pub memory_mb: f64,
    pub memory_percent: f64,
    pub worker_count: u64,
    pub failed_workers: u64,
}

#[derive(Debug, Clone, Default)]
pub struct WorkerHealthTelemetry {
    pub timestamp_ns: u64,
    pub worker_name: String,
    pub priority: u8,
    pub running: bool,
    pub failed: bool,
    pub uptime_seconds: f64,
}

#[derive(Debug, Clone, Default)]
pub struct StorageHealthTelemetry {
    pub timestamp_ns: u64,
    pub disk_free_gb: f64,
    pub raw_bytes_written: u64,
    pub raw_messages_written: u64,
    pub pcd_points_written: u64,
    pub pcd_frames_written: u64,
    pub write_errors: u64,
}

#[derive(Debug, Clone, Default)]
pub struct PowerThermalTelemetry {
    pub timestamp_ns: u64,
    pub available: bool,
    pub cpu_temperature_c: f64,
    pub gpu_temperature_c: f64,
    pub board_power_w: f64,
    pub fan_rpm: f64,
}
