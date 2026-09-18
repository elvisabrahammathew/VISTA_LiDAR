//! Application-level capture and point-cloud processing modules.

// The common application entry point keeps callers independent of the physical
// processing/object subfolders.
#[path = "object/analytics/analytics.rs"]
pub mod analytics;
#[path = "object/detection/detection.rs"]
pub mod detection;
#[path = "grafana_bridge/grafana_bridge.rs"]
pub mod grafana_bridge;
#[path = "logger/logger.rs"]
pub mod logger;
#[path = "processing/pointcloud/pointcloud_processing.rs"]
pub mod pointcloud_processing;
#[path = "monitoring/system_monitor.rs"]
pub mod system_monitor;
#[path = "object/tracking/tracking.rs"]
pub mod tracking;
