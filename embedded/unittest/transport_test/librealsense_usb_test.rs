//! Unit tests for the native librealsense USB transport.

use super::native::runtime_api_version;

/// Confirms that the bundled runtime exposes the exact API used by these bindings.
#[test]
fn loads_librealsense_2_50_runtime() {
    assert_eq!(runtime_api_version().unwrap(), 25_000);
}
