//! Unit tests for DeviceConfig.txt parsing and runtime defaults.

use super::*;

const QUANERGY_CONFIG: &str = "\
Lidar: quanergy-m8
ReconnectIntervalSeconds(Lidar): 7
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
Radar: None
";

/// Confirms that all Quanergy connection values are read from the text file.
#[test]
fn parses_quanergy_settings() {
    let config = parse_device_config(QUANERGY_CONFIG).unwrap();
    assert_eq!(config.lidar_type, LidarType::QuanergyM8);
    assert_eq!(config.sensor_ip, "192.168.1.20".parse::<IpAddr>().unwrap());
    assert_eq!(config.tcp_port, Some(5000));
    assert_eq!(config.lidar_reconnect_interval, Duration::from_secs(7));
    assert!(!config.raw_logging_enabled);
    assert!(!config.pointcloud_logging_enabled);
}

/// RAW and PCD logging can be enabled independently for the selected LiDAR.
#[test]
fn parses_independent_lidar_logging_switches() {
    let contents = QUANERGY_CONFIG.replace(
        "ReconnectIntervalSeconds(Lidar): 7",
        "RawLoggingEnabled(Lidar): 1\nPointCloudLoggingEnabled(Lidar): 0\nReconnectIntervalSeconds(Lidar): 7",
    );
    let config = parse_device_config(&contents).unwrap();
    assert!(config.raw_logging_enabled);
    assert!(!config.pointcloud_logging_enabled);

    let runtime = config.runtime_config().unwrap();
    assert!(runtime.threads.raw_logger.enabled);
    assert!(!runtime.threads.pcd_logger.enabled);
}

#[test]
fn rejects_invalid_lidar_logging_switch() {
    let contents = QUANERGY_CONFIG.replace(
        "ReconnectIntervalSeconds(Lidar): 7",
        "RawLoggingEnabled(Lidar): 2\nReconnectIntervalSeconds(Lidar): 7",
    );
    let error = parse_device_config(&contents).unwrap_err();
    assert!(error.contains("RawLoggingEnabled(Lidar)"));
}

/// An empty serial means that librealsense may select the first matching L515.
#[test]
fn accepts_blank_realsense_serial() {
    let config = parse_device_config(
        "Lidar: realsense-l515\nUsbSerial(realsensel515):\nDepthWidth(realsensel515): 640\nDepthHeight(realsensel515): 480\nDepthFps(realsensel515): 30\nRadar: None\n",
    )
    .unwrap();
    assert_eq!(config.lidar_type, LidarType::RealSenseL515);
    assert_eq!(config.usb_serial, None);
    assert_eq!(
        (config.depth_width, config.depth_height, config.depth_fps),
        (640, 480, 30)
    );
}

/// A RealSense serial is an SDK device identifier, not a Windows COM port.
#[test]
fn preserves_realsense_serial() {
    let config = parse_device_config(
        "Lidar: realsense-l515\nUsbSerial(realsensel515): 123456789012\nDepthWidth(realsensel515): 1024\nDepthHeight(realsensel515): 768\nDepthFps(realsensel515): 15\nRadar:\n",
    )
    .unwrap();
    assert_eq!(config.usb_serial.as_deref(), Some("123456789012"));
    assert_eq!(
        (config.depth_width, config.depth_height, config.depth_fps),
        (1024, 768, 15)
    );
}

/// Keeps the earlier ComPort spelling as a compatibility alias.
#[test]
fn accepts_legacy_com_port_key() {
    let config = parse_device_config(
        "Lidar: realsense-l515\nComPort(realsensel515): TEST-SERIAL\nDepthWidth: 640\nDepthHeight: 480\nDepthFps: 30\nRadar: None\n",
    )
    .unwrap();
    assert_eq!(config.usb_serial.as_deref(), Some("TEST-SERIAL"));
}

#[test]
fn requires_lidar_selection() {
    let error = parse_device_config("Radar: None\n").unwrap_err();
    assert!(error.contains("missing a Lidar selection"));
}

#[test]
fn requires_quanergy_network_settings() {
    let error = parse_device_config("Lidar: quanergy-m8\nRadar: None\n").unwrap_err();
    assert!(error.contains("requires SensorIP and TcpPort"));
}

#[test]
fn rejects_unimplemented_radar() {
    let contents = QUANERGY_CONFIG.replace("Radar: None", "Radar: example-radar");
    let error = parse_device_config(&contents).unwrap_err();
    assert!(error.contains("Radar is not implemented"));
}

#[test]
fn rejects_removed_duration_setting() {
    let error =
        parse_device_config(&format!("{QUANERGY_CONFIG}DurationSeconds: 10\n")).unwrap_err();
    assert!(error.contains("unknown device key 'durationseconds'"));
}

#[test]
fn reconnect_interval_must_be_positive() {
    let contents = QUANERGY_CONFIG.replace(
        "ReconnectIntervalSeconds(Lidar): 7",
        "ReconnectIntervalSeconds(Lidar): 0",
    );
    let error = parse_device_config(&contents).unwrap_err();
    assert!(error.contains("must be between 1"));
}

/// Confirms priority and bounded queue defaults used by main.
#[test]
fn runtime_defaults_match_worker_architecture() {
    let runtime = parse_device_config(QUANERGY_CONFIG)
        .unwrap()
        .runtime_config()
        .unwrap();
    assert_eq!(runtime.threads.lidar_read.thread.priority.level(), 1);
    assert_eq!(runtime.threads.lidar_decode.thread.priority.level(), 2);
    assert_eq!(runtime.threads.preprocessing.thread.priority.level(), 3);
    assert_eq!(runtime.queues.raw_capacity, 32);
    assert_eq!(runtime.queues.decoded_capacity, 8);
    assert_eq!(runtime.queues.processed_capacity, 8);
}
