//! Selects the platform implementation at compile time.

// Give each operating-system module its own name. This is easier for IDEs to
// analyze than declaring two conditional modules with the same name `current`.
#[cfg(target_os = "linux")]
#[path = "os/linux.rs"]
mod linux;

#[cfg(target_os = "windows")]
#[path = "os/window.rs"]
mod windows;

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
compile_error!("this capture application currently supports only Windows and Linux");

// Expose one stable platform API to the device drivers.
#[cfg(target_os = "linux")]
pub use linux::{configure_sensor_stream, PLATFORM_NAME};

#[cfg(target_os = "windows")]
pub use windows::{configure_sensor_stream, PLATFORM_NAME};
