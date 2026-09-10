//! Application-level capture and point-cloud processing modules.

// The common application entry point keeps callers independent of the physical
// processing/object subfolders.
#[path = "object/analytics.rs"]
pub mod analytics;
#[path = "object/detection.rs"]
pub mod detection;
#[path = "pipeline.rs"]
pub mod pipeline;
#[path = "processing/pointcloud_processing.rs"]
pub mod pointcloud_processing;
#[path = "object/tracking.rs"]
pub mod tracking;
