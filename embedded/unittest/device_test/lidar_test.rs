//! Unit tests for the common lidar.rs facade and configuration.

use super::*;

/// Verifies the short and full Quanergy configuration names.
#[test]
fn parses_quanergy_aliases() {
    assert_eq!("quanergy-m8".parse(), Ok(LidarType::QuanergyM8));
    assert_eq!("m8".parse(), Ok(LidarType::QuanergyM8));
}

/// Verifies that unknown LiDAR names produce a useful configuration error.
#[test]
fn rejects_unknown_lidar_type() {
    assert!("unknown-lidar".parse::<LidarType>().is_err());
}
