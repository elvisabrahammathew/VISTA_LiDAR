#include <chrono>
#include <ctime>
#include <string>

#include "config.hpp"
#include "unittest/test.hpp"

namespace {

const char* quanergy_config = R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 7
SensorIP(quanergy-m8, unitree-l2): 192.168.1.20
SensorPort(quanergy-m8, unitree-l2): 5000
Radar: None
)";

}  // namespace

VISTA_TEST(config_file_parses_quanergy_network_settings_and_notes) {
    const auto config = vista::parse_device_config_text(quanergy_config);

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::quanergy_m8);
    VISTA_CHECK(config.sensor_ip == "192.168.1.20");
    VISTA_CHECK(config.sensor_port == 5000);
    VISTA_CHECK(config.lidar_reconnect_interval == std::chrono::seconds(7));
    VISTA_CHECK(!config.raw_logging_enabled);
    VISTA_CHECK(!config.pointcloud_logging_enabled);
}

VISTA_TEST(config_file_parses_independent_lidar_logging_switches) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
RawLoggingEnabled(Lidar): 1
PointCloudLoggingEnabled(Lidar): 0
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
)");

    VISTA_CHECK(config.raw_logging_enabled);
    VISTA_CHECK(!config.pointcloud_logging_enabled);
    const auto runtime = config.runtime_config();
    VISTA_CHECK(runtime.threads.raw_logger.enabled);
    VISTA_CHECK(!runtime.threads.pcd_logger.enabled);
}

VISTA_TEST(config_file_rejects_invalid_lidar_logging_switch) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergy-m8
RawLoggingEnabled(Lidar): 2
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
)") );
}

VISTA_TEST(config_file_parses_realsense_with_first_matching_device) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsensel515
UsbSerial(realsensel515):
DepthWidth(realsensel515): 1024
DepthHeight(realsensel515): 768
DepthFps(realsensel515): 30
Radar:
)");

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::realsense_l515);
    VISTA_CHECK(!config.usb_serial);
    VISTA_CHECK(config.depth_width == 1024);
    VISTA_CHECK(config.depth_height == 768);
    VISTA_CHECK(config.depth_fps == 30);
}

VISTA_TEST(config_file_parses_explicit_realsense_usb_serial) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsense-l515
UsbSerial(realsensel515): 123456789
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30
)");

    VISTA_CHECK(config.usb_serial == std::string("123456789"));
}

VISTA_TEST(config_file_parses_unitree_auto_serial_and_udp_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: unitree-l2
SensorIP(quanergy-m8, unitree-l2): 192.168.1.62
SensorPort(quanergy-m8, unitree-l2): 6101
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
)");

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::unitree_l2);
    VISTA_CHECK(config.sensor_ip == "192.168.1.62");
    VISTA_CHECK(config.sensor_port == 6101);
    VISTA_CHECK(
        config.unitree_connection_mode ==
        vista::devices::UnitreeConnectionMode::automatic);
    VISTA_CHECK(config.unitree_local_ip == "192.168.1.2");
    VISTA_CHECK(config.unitree_local_port == 6201);
    VISTA_CHECK(config.unitree_serial_port == "COM3");
    VISTA_CHECK(config.unitree_baud_rate == 4'000'000);

    const auto lidar = config.lidar_config();
    VISTA_CHECK(lidar.unitree_l2.has_value());
    VISTA_CHECK(lidar.unitree_l2->sensor_port == 6101);
    VISTA_CHECK(lidar.unitree_l2->local_port == 6201);
}

VISTA_TEST(config_file_parses_mounting_ground_and_imu_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: unitree-l2
SensorIP(quanergy-m8, unitree-l2): 192.168.1.62
SensorPort(quanergy-m8, unitree-l2): 6101
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
GroundMode(lidar): hybrid
UseImuForGround(lidar): 1
MountX(lidar): -1.25
MountY(lidar): 2.5
MountZ(lidar): 0.75
MountRollDeg(lidar): 1.5
MountPitchDeg(lidar): -2.5
MountYawDeg(lidar): 90.0
FloorZ(lidar): 0.0
GroundDistanceThreshold(lidar): 0.10
GroundNormalToleranceDeg(lidar): 15.0
GroundCalibrationFrames(lidar): 50
GroundMinInlierRatio(lidar): 0.20
)");

    VISTA_CHECK(config.ground_removal.has_value());
    const auto& ground = *config.ground_removal;
    VISTA_CHECK(ground.mode == vista::application::GroundMode::hybrid);
    VISTA_CHECK(ground.use_imu);
    VISTA_CHECK_NEAR(ground.mount_x_m, -1.25F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_y_m, 2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_z_m, 0.75F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_roll_deg, 1.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_pitch_deg, -2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_yaw_deg, 90.0F, 1.0e-6F);
    VISTA_CHECK(ground.calibration_frames == 50);
    VISTA_CHECK_NEAR(ground.minimum_inlier_ratio, 0.20F, 1.0e-6F);
    VISTA_CHECK(config.runtime_config().preprocessing.ground_removal.has_value());
}

VISTA_TEST(config_file_accepts_all_ground_modes) {
    VISTA_CHECK(
        vista::application::parse_ground_mode("static") ==
        vista::application::GroundMode::static_height);
    VISTA_CHECK(
        vista::application::parse_ground_mode("ransac") ==
        vista::application::GroundMode::ransac);
    VISTA_CHECK(
        vista::application::parse_ground_mode("hybrid") ==
        vista::application::GroundMode::hybrid);
    VISTA_CHECK_THROWS(vista::application::parse_ground_mode("automatic"));
}

VISTA_TEST(config_file_accepts_com_port_as_legacy_usb_serial_name) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsensel515
ComPort(realsensel515): 987654321
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30
)");

    VISTA_CHECK(config.usb_serial == std::string("987654321"));
}

VISTA_TEST(config_file_requires_lidar) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        "Radar: None\n"));
}

VISTA_TEST(config_file_requires_quanergy_network_settings) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        "Lidar: quanergym8\n"));
}

VISTA_TEST(config_file_rejects_unimplemented_radar) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergym8
Radar: example-radar
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
)"));
}

VISTA_TEST(config_file_rejects_removed_duration_setting) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        std::string(quanergy_config) + "DurationSeconds: 10\n"));
}

VISTA_TEST(config_file_rejects_zero_lidar_reconnect_interval) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 0
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
)"));
}

VISTA_TEST(config_formats_log_timestamp_in_local_system_time) {
    std::tm local_time{};
    local_time.tm_year = 2026 - 1900;
    local_time.tm_mon = 9 - 1;
    local_time.tm_mday = 16;
    local_time.tm_hour = 13;
    local_time.tm_min = 14;
    local_time.tm_sec = 5;
    local_time.tm_isdst = -1;
    const auto raw_time = std::mktime(&local_time);
    VISTA_CHECK(raw_time != static_cast<std::time_t>(-1));

    VISTA_CHECK(
        vista::format_log_timestamp(
            std::chrono::system_clock::from_time_t(raw_time)) ==
        "20260916_131405");
}

VISTA_TEST(config_exposes_independent_worker_enable_and_priority_settings) {
    const auto config = vista::parse_device_config_text(quanergy_config);
    const auto runtime = config.runtime_config();

    VISTA_CHECK(runtime.threads.lidar_read.enabled);
    VISTA_CHECK(runtime.threads.lidar_read.thread.priority.level() == 1);
    VISTA_CHECK(runtime.threads.lidar_decode.thread.priority.level() == 2);
    VISTA_CHECK(runtime.threads.preprocessing.thread.priority.level() == 3);
    VISTA_CHECK(runtime.threads.grafana_bridge.thread.priority.level() == 4);
    VISTA_CHECK(
        runtime.threads.pointcloud_websocket.thread.priority.level() == 4);
    VISTA_CHECK(runtime.threads.system_monitor.thread.priority.level() == 4);
    VISTA_CHECK(runtime.queues.raw_capacity == 32);
    VISTA_CHECK(runtime.queues.telemetry_capacity == 16);
}

VISTA_TEST(config_file_parses_pointcloud_websocket_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
PointCloudWebSocketEnabled: true
PointCloudWebSocketBindAddress: 0.0.0.0
PointCloudWebSocketPort: 9876
PointCloudWebSocketMaxPoints: 75000
PointCloudWebSocketMaxClients: 3
PointCloudWebSocketPublishIntervalMilliseconds: 125
PointCloudWebSocketRetryIntervalSeconds: 4
)");

    VISTA_CHECK(config.pointcloud_websocket.enabled);
    VISTA_CHECK(config.pointcloud_websocket.bind_address == "0.0.0.0");
    VISTA_CHECK(config.pointcloud_websocket.port == 9876);
    VISTA_CHECK(config.pointcloud_websocket.maximum_points == 75'000);
    VISTA_CHECK(config.pointcloud_websocket.maximum_clients == 3);
    VISTA_CHECK(
        config.pointcloud_websocket.publish_interval ==
        std::chrono::milliseconds(125));
    VISTA_CHECK(
        config.pointcloud_websocket.retry_interval ==
        std::chrono::seconds(4));
    VISTA_CHECK(config.runtime_config().threads.pointcloud_websocket.enabled);
}

VISTA_TEST(config_file_parses_grafana_live_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
GrafanaEnabled: yes
GrafanaHost: localhost
GrafanaPort: 3100
GrafanaNamespace: factory_floor
GrafanaPublishIntervalMilliseconds: 250
GrafanaRetryIntervalSeconds: 9
SystemMonitorIntervalMilliseconds: 1500
)");

    VISTA_CHECK(config.grafana.enabled);
    VISTA_CHECK(config.grafana.host == "localhost");
    VISTA_CHECK(config.grafana.port == 3100);
    VISTA_CHECK(config.grafana.name_space == "factory_floor");
    VISTA_CHECK(config.grafana.publish_interval == std::chrono::milliseconds(250));
    VISTA_CHECK(config.grafana.retry_interval == std::chrono::seconds(9));
    VISTA_CHECK(
        config.system_monitor.sample_interval == std::chrono::milliseconds(1500));
}

VISTA_TEST(config_file_rejects_invalid_grafana_namespace) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
GrafanaNamespace: invalid/name
)") );
}
