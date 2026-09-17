//! Shared entry point for data transport and output implementations.

// Keep the concrete transports behind one common layer so other layers import
// `crate::transport` instead of depending on the physical folder layout.
#[path = "peripheral/ethernet/ethernet.rs"]
pub mod ethernet;
#[path = "messaging/http.rs"]
pub mod http;
#[path = "peripheral/librealsense_usb/librealsense_usb.rs"]
pub mod librealsense_usb;
#[path = "storage/local.rs"]
pub mod local;
#[path = "messaging/mqtt.rs"]
pub mod mqtt;
