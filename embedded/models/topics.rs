//! Stable topic names shared across the whole process.

// LiDAR acquisition and point-cloud processing topics.
pub const LIDAR_RAW: &str = "lidar/raw";
pub const POINTCLOUD_DECODED: &str = "pointcloud/decoded";
pub const POINTCLOUD_PROCESSED: &str = "pointcloud/processed";

// Platform and application monitoring topics consumed by grafana_bridge.
pub const SYSTEM_HEALTH: &str = "system/health";
pub const WORKER_HEALTH: &str = "worker/health";
pub const STORAGE_HEALTH: &str = "storage/health";
pub const POWER_THERMAL: &str = "power/thermal";
