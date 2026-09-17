#include <chrono>
#include <ctime>
#include <string>

#include "config.hpp"
#include "unittest/test.hpp"

namespace {

const char* quanergy_config = R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 7
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
Radar: None
)";

}  // namespace

VISTA_TEST(config_file_parses_quanergy_network_settings_and_notes) {
    const auto config = vista::parse_device_config_text(quanergy_config);

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::quanergy_m8);
    VISTA_CHECK(config.sensor_ip == "192.168.1.20");
    VISTA_CHECK(config.tcp_port == 5000);
    VISTA_CHECK(config.lidar_reconnect_interval == std::chrono::seconds(7));
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
    VISTA_CHECK(runtime.queues.raw_capacity == 32);
}
