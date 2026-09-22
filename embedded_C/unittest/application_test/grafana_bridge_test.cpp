#include <string>

#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "unittest/test.hpp"

VISTA_TEST(grafana_bridge_formats_pointcloud_as_influx_line_protocol) {
    const vista::application::PointCloudTelemetry telemetry{
        "quanergy m8,roof",
        1'725'000'000'000'000'000ULL,
        12'000,
        3'250,
        2,
        19.5,
        4.25,
        true,
        true,
        -10.0F,
        12.0F,
        -4.0F,
        5.0F,
        -2.5F,
        1.5F,
    };

    const auto line =
        vista::application::format_pointcloud_measurement(telemetry);

    VISTA_CHECK(line.find("pointcloud,lidar_id=quanergy\\ m8\\,roof") == 0);
    VISTA_CHECK(line.find("online=1i") != std::string::npos);
    VISTA_CHECK(line.find("input_point_count=12000i") != std::string::npos);
    VISTA_CHECK(line.find("point_count=3250i") != std::string::npos);
    VISTA_CHECK(line.find("min_z_m=-2.5") != std::string::npos);
    VISTA_CHECK(
        line.find(" 1725000000000000000") != std::string::npos);
}

VISTA_TEST(grafana_bridge_formats_imu_as_influx_line_protocol) {
    vista::application::ImuTelemetry telemetry;
    telemetry.lidar_id = "unitree-l2";
    telemetry.timestamp_ns = 1'725'000'000'000'000'001ULL;
    telemetry.sample_rate_hz = 200.0;
    telemetry.online = true;
    telemetry.sample.orientation_x = 0.1F;
    telemetry.sample.orientation_w = 0.9F;
    telemetry.sample.angular_velocity_z_rad_s = 1.3F;
    telemetry.sample.linear_acceleration_x_m_s2 = 9.7F;

    const auto line = vista::application::format_imu_measurement(telemetry);

    VISTA_CHECK(line.find("imu,lidar_id=unitree-l2") == 0);
    VISTA_CHECK(line.find("online=1i") != std::string::npos);
    VISTA_CHECK(line.find("sample_rate_hz=200") != std::string::npos);
    VISTA_CHECK(line.find("orientation_x=") != std::string::npos);
    VISTA_CHECK(line.find("angular_velocity_z_rad_s=") != std::string::npos);
    VISTA_CHECK(line.find("linear_acceleration_x_m_s2=") != std::string::npos);
}

VISTA_TEST(grafana_bridge_rejects_invalid_channel_namespace) {
    auto config = vista::application::GrafanaBridgeConfig{};
    config.name_space = "factory/line";
    VISTA_CHECK_THROWS(
        vista::application::validate_grafana_bridge_config(config));
}

VISTA_TEST(grafana_bridge_formats_service_account_authorization) {
    VISTA_CHECK(
        vista::application::make_grafana_bearer_authorization("glsa_test-token") ==
        "Bearer glsa_test-token");
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization(""));
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization("glsa_bad token"));
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization("token-name-only"));
}
