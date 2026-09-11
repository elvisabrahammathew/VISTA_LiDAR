//! Unit tests for main.rs command-line configuration.

use super::*;

/// Confirms that an empty command uses defaults and disables both files.
#[test]
fn empty_command_uses_defaults_without_outputs() {
    let config = parse_arguments(&[]).unwrap();
    assert_eq!(config.lidar_type, LidarType::QuanergyM8);
    assert_eq!(config.sensor_ip, DEFAULT_SENSOR_IP);
    assert_eq!(config.tcp_port, None);
    assert_eq!(config.duration_seconds, 10);
    assert!(config.raw_path.is_none());
    assert!(config.pcd_path.is_none());
}

/// Confirms that named options can omit IP and port while overriding other values.
#[test]
fn named_options_allow_missing_ip_and_port() {
    let args = vec![
        "--duration".to_owned(),
        "20".to_owned(),
        "--pcd".to_owned(),
        "cloud.pcd".to_owned(),
    ];
    let config = parse_arguments(&args).unwrap();
    assert_eq!(config.sensor_ip, DEFAULT_SENSOR_IP);
    assert_eq!(config.tcp_port, None);
    assert_eq!(config.duration_seconds, 20);
    assert_eq!(config.pcd_path, Some(PathBuf::from("cloud.pcd")));
    assert!(config.raw_path.is_none());
}

/// Confirms that the complete positional command overrides every capture value.
#[test]
fn full_positional_command_uses_provided_values() {
    let args = vec![
        "quanergy-m8".to_owned(),
        "192.168.1.20".to_owned(),
        "5000".to_owned(),
        "30".to_owned(),
        "capture.bin".to_owned(),
        "cloud.pcd".to_owned(),
    ];
    let config = parse_arguments(&args).unwrap();
    assert_eq!(config.sensor_ip, "192.168.1.20".parse::<IpAddr>().unwrap());
    assert_eq!(config.tcp_port, Some(5000));
    assert_eq!(config.duration_seconds, 30);
    assert_eq!(config.raw_path, Some(PathBuf::from("capture.bin")));
    assert_eq!(config.pcd_path, Some(PathBuf::from("cloud.pcd")));
}
