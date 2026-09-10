//! Shared entry point for data transport and output implementations.

// Keep the concrete transports behind one common layer so applications import
// `crate::transport` instead of depending on the physical folder layout.
#[path = "messaging/http.rs"]
pub mod http;
#[path = "storage/local.rs"]
pub mod local;
#[path = "messaging/mqtt.rs"]
pub mod mqtt;
