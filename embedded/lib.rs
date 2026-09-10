//! Core library for capturing and decoding LiDAR sensor data.

// Each numbered folder is one architecture layer. The files referenced here
// are the shared entry points that expose that layer to the rest of the crate.
#[path = "4_Applications/application.rs"]
pub mod application;
#[path = "3_Devices/devices.rs"]
pub mod devices;
#[path = "models/models.rs"]
pub mod models;
#[path = "1_Platform/platform.rs"]
pub mod platform;
#[path = "2_Transport/transport.rs"]
pub mod transport;
