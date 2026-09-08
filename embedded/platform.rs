//! Selects the platform implementation at compile time.

// Only the matching module is compiled for the current target OS.
#[cfg(target_os = "linux")]
#[path = "platform/linux.rs"]
mod current;

#[cfg(target_os = "windows")]
#[path = "platform/window.rs"]
mod current;

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
compile_error!("this capture application currently supports only Windows and Linux");

pub use current::{configure_sensor_stream, PLATFORM_NAME};
