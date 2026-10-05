#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include "1_Platform/compat/filesystem.hpp"
#include <optional>
#include <string>

#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "4_Applications/monitoring/system_monitor.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include "4_Applications/mapping/room_map/room_map.hpp"
#include "4_Applications/mapping/lio/lio.hpp"

namespace vista {

inline constexpr const char* default_device_config_path = "DeviceConfig.txt";

/// Lets main decide which independent workers exist in this application run.
struct WorkerConfig {
    bool enabled{true};
    platform::ThreadConfig thread;
};

struct ThreadSetConfig {
    WorkerConfig lidar_read;
    WorkerConfig lidar_decode;
    WorkerConfig raw_logger;
    WorkerConfig preprocessing;
    WorkerConfig pcd_logger;
    WorkerConfig system_monitor;
    WorkerConfig grafana_bridge;
    WorkerConfig pointcloud_websocket;
    WorkerConfig ground_processing;
    WorkerConfig room_mapping;
    WorkerConfig room_map_websocket;
    WorkerConfig localization;
    WorkerConfig live_preprocessing;
};

struct TopicQueueConfig {
    std::size_t raw_capacity{};
    std::size_t decoded_capacity{};
    std::size_t processed_capacity{};
    std::size_t telemetry_capacity{};
};

struct RuntimeConfig {
    application::PreprocessingConfig preprocessing;
    ThreadSetConfig threads;
    TopicQueueConfig queues;
};

struct AppConfig {
    devices::LidarType lidar_type{devices::LidarType::quanergy_m8};
    std::string sensor_ip{"192.168.1.3"};
    std::optional<std::uint16_t> sensor_port;
    std::optional<std::string> usb_serial;
    std::uint32_t depth_width{640};
    std::uint32_t depth_height{480};
    std::uint32_t depth_fps{30};
    devices::UnitreeConnectionMode unitree_connection_mode{
        devices::UnitreeConnectionMode::automatic};
#ifdef _WIN32
    std::string unitree_serial_port{"COM3"};
#else
    std::string unitree_serial_port{"/dev/ttyACM0"};
#endif
    std::uint32_t unitree_baud_rate{4'000'000};
    std::string unitree_local_ip{"192.168.1.2"};
    std::optional<std::uint16_t> unitree_local_port;
    /// Controls whether the RAW logger worker and .bin output are created.
    bool raw_logging_enabled{false};
    /// Controls whether the PCD logger worker and .pcd output are created.
    bool pointcloud_logging_enabled{false};
    /// Delay before retrying while the selected LiDAR is unavailable.
    std::chrono::milliseconds lidar_reconnect_interval{5'000};
    application::GrafanaBridgeConfig grafana;
    /// Both WebSocket enabled flags are derived from grafana.enabled, not separate TXT switches.
    application::PointCloudWebSocketConfig pointcloud_websocket = [] {
        application::PointCloudWebSocketConfig value;
        value.input_topic = "pointcloud/cleaned_sensor";
        // A world ground plane must never be drawn over sensor-coordinate XYZ.
        value.include_ground_status = false;
        return value;
    }();
    application::RoomMapConfig room_map;
    application::LioConfig lio;
    application::PointCloudWebSocketConfig room_map_websocket = [] {
        application::PointCloudWebSocketConfig value;
        value.enabled = false; // AppConfig parsing derives this from GrafanaEnabled.
        value.port = 8766;
        value.maximum_points = 100'000;
        // Share the map's default too; parsing derives any TXT override below.
        value.publish_interval = application::RoomMapConfig{}.publish_interval;
        value.input_topic = "mapping/room_map";
        value.retained_snapshot = true;
        value.room_map_lod = true;
        value.include_ground_status = false;
        return value;
    }();
    application::SystemMonitorConfig system_monitor;
    /// Enabled when GroundMode is present in DeviceConfig.txt.
    std::optional<application::GroundRemovalConfig> ground_removal;
    /// Common mounting transform is independent of the ground-processing branch.
    std::optional<application::GroundRemovalConfig> mounting;
    std::optional<vista::fs::path> raw_path;
    std::optional<vista::fs::path> pcd_path;

    static AppConfig load();
    devices::LidarConfig lidar_config() const;
    RuntimeConfig runtime_config() const;
};

// Public for focused unit tests; normal application code should call AppConfig::load.
AppConfig parse_device_config_text(const std::string& contents);
std::string format_log_timestamp(
    std::chrono::system_clock::time_point timestamp);
/// Uses ONE session timestamp for RAW, processed PCD, and the new room-map name.
/// RoomMapFile is an input path only in read-only loading mode.
void configure_session_output_paths(AppConfig& config,
    const vista::fs::path& data_root,
    std::chrono::system_clock::time_point session_start);

}  // namespace vista
