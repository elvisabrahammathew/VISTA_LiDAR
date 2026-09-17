//! Device interfaces and sensor-specific drivers.

// `lidar` is the common facade used by main and the application layer.
#[path = "lidars/lidar.rs"]
pub mod lidar;
#[path = "lidars/quanergym8/quanergym8.rs"]
pub mod quanergym8;
#[path = "lidars/realsensel515/realsensel515.rs"]
pub mod realsensel515;
#[path = "lidars/unitree4d/unitree4d.rs"]
pub mod unitree4d;
