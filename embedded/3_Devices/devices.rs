//! Device interfaces and sensor-specific drivers.

// `lidar` is the common facade used by main and the application layer.
#[path = "lidars/lidar.rs"]
pub mod lidar;
#[path = "lidars/lidar_quanergym8.rs"]
pub mod lidar_quanergym8;
#[path = "lidars/lidar_unitree4d.rs"]
pub mod lidar_unitree4d;
