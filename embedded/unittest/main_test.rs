//! Unit tests for main.rs command-line configuration.

use super::*;

/// Confirms that an empty command uses defaults and disables both files.
#[test]
fn empty_command_uses_defaults_without_outputs() {
    let config = parse_arguments(&[], LidarType::QuanergyM8).unwrap();
    assert_eq!(config.lidar_type, LidarType::QuanergyM8);
    assert_eq!(config.sensor_ip, DEFAULT_SENSOR_IP);
    assert_eq!(config.tcp_port, None);
    assert_eq!(config.duration_seconds, 10);
    assert_eq!(config.depth_width, DEFAULT_DEPTH_WIDTH);
    assert_eq!(config.depth_height, DEFAULT_DEPTH_HEIGHT);
    assert_eq!(config.depth_fps, DEFAULT_DEPTH_FPS);
    assert!(config.raw_path.is_none());
    assert!(config.pcd_path.is_none());
}

/// Confirms that an L515 command accepts depth resolution and frame-rate overrides.
#[test]
fn named_realsense_options_configure_depth_stream() {
    let args = vec![
        "--width".to_owned(),
        "1024".to_owned(),
        "--height".to_owned(),
        "768".to_owned(),
        "--fps".to_owned(),
        "30".to_owned(),
    ];
    let config = parse_arguments(&args, LidarType::RealSenseL515).unwrap();
    assert_eq!(config.lidar_type, LidarType::RealSenseL515);
    assert_eq!(config.depth_width, 1024);
    assert_eq!(config.depth_height, 768);
    assert_eq!(config.depth_fps, 30);
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
    let config = parse_arguments(&args, LidarType::QuanergyM8).unwrap();
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
        "192.168.1.20".to_owned(),
        "5000".to_owned(),
        "30".to_owned(),
        "capture.bin".to_owned(),
        "cloud.pcd".to_owned(),
    ];
    let config = parse_arguments(&args, LidarType::QuanergyM8).unwrap();
    assert_eq!(config.sensor_ip, "192.168.1.20".parse::<IpAddr>().unwrap());
    assert_eq!(config.tcp_port, Some(5000));
    assert_eq!(config.duration_seconds, 30);
    assert_eq!(config.raw_path, Some(PathBuf::from("capture.bin")));
    assert_eq!(config.pcd_path, Some(PathBuf::from("cloud.pcd")));
}

/// Confirms that the device file selects Quanergy and permits an unused Radar entry.
#[test]
fn device_config_selects_quanergy_and_ignores_radar() {
    let lidar = parse_device_config("Lidar: quanergy-m8\nRadar: None\n").unwrap();
    assert_eq!(lidar, LidarType::QuanergyM8);
}

/// Confirms that blank Radar configuration is accepted for future implementation.
#[test]
fn device_config_selects_realsense_with_blank_radar() {
    let lidar = parse_device_config("Lidar: realsense-l515\nRadar:\n").unwrap();
    assert_eq!(lidar, LidarType::RealSenseL515);
}

/// Confirms that a missing LiDAR selection produces a clear configuration error.
#[test]
fn device_config_requires_lidar_entry() {
    let error = parse_device_config("Radar: None\n").unwrap_err();
    assert!(error.contains("missing 'Lidar:"));
}

/// Confirms that the LiDAR can no longer be overridden from the command line.
#[test]
fn command_line_rejects_lidar_override() {
    let args = vec!["--lidar".to_owned(), "l515".to_owned()];
    let error = parse_arguments(&args, LidarType::QuanergyM8).unwrap_err();
    assert!(error.contains("unknown option '--lidar'"));
}
