//! Focused tests for Grafana line protocol and authentication formatting.

use super::*;

#[test]
fn formats_pointcloud_as_influx_line_protocol() {
    let value = PointCloudTelemetry {
        lidar_id: "quanergy m8,roof".to_owned(),
        timestamp_ns: 1_725_000_000_000_000_000,
        input_point_count: 12_000,
        point_count: 3_250,
        dropped_messages: 2,
        frames_per_second: 19.5,
        processing_ms: 4.25,
        online: true,
        bounds: Some([-10.0, 12.0, -4.0, 5.0, -2.5, 1.5]),
    };
    let line = format_pointcloud_measurement(&value);
    assert!(line.starts_with("pointcloud,lidar_id=quanergy\\ m8\\,roof"));
    assert!(line.contains("online=1i"));
    assert!(line.contains("point_count=3250i"));
    assert!(line.ends_with("1725000000000000000"));
}

#[test]
fn validates_service_account_token() {
    assert_eq!(
        make_grafana_bearer_authorization("glsa_test-token").unwrap(),
        "Bearer glsa_test-token"
    );
    assert!(make_grafana_bearer_authorization("token-name").is_err());
    assert!(make_grafana_bearer_authorization("glsa_bad token").is_err());
}

#[test]
fn rejects_invalid_namespace() {
    let config = GrafanaBridgeConfig {
        namespace: "factory/line".to_owned(),
        ..GrafanaBridgeConfig::default()
    };
    assert!(validate_grafana_bridge_config(&config).is_err());
}
