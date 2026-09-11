//! Unit tests for RealSense L515 Z16-to-point-cloud conversion.

use super::native::{decode_depth_frame, runtime_api_version};

/// Confirms that the bundled DLL exposes the exact API used by these bindings.
#[test]
fn loads_librealsense_2_50_runtime() {
    assert_eq!(runtime_api_version().unwrap(), 25_000);
}

/// Confirms that zero depth pixels are excluded from the point cloud.
#[test]
fn ignores_invalid_zero_depth_pixels() {
    let bytes = [0_u8, 0_u8];
    let frame = decode_depth_frame(&bytes, &[[0.0, 0.0, 1.0]], 0.001, 10).unwrap();
    assert!(frame.points.is_empty());
    assert_eq!(frame.timestamp_ns, 10);
}

/// Confirms conversion from RealSense optical axes to the application axes.
#[test]
fn converts_optical_coordinates_to_application_coordinates() {
    let bytes = 1_000_u16.to_le_bytes();
    let frame = decode_depth_frame(&bytes, &[[0.25, 0.5, 1.0]], 0.001, 20).unwrap();
    assert_eq!(frame.points.len(), 1);

    let point = frame.points[0];
    assert!((point.x - 1.0).abs() < 1.0e-6);
    assert!((point.y + 0.25).abs() < 1.0e-6);
    assert!((point.z + 0.5).abs() < 1.0e-6);
    assert_eq!(point.timestamp_ns, 20);
}

/// Confirms that truncated or oversized depth buffers are rejected.
#[test]
fn rejects_incorrect_depth_buffer_size() {
    assert!(decode_depth_frame(&[1], &[[0.0, 0.0, 1.0]], 0.001, 0).is_err());
}
