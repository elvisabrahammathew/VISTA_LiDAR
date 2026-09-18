#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "4_Applications/monitoring/system_monitor.hpp"
#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"

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
    std::optional<std::uint16_t> tcp_port;
    std::optional<std::string> usb_serial;
    std::uint32_t depth_width{640};
    std::uint32_t depth_height{480};
    std::uint32_t depth_fps{30};
    /// Controls whether the RAW logger worker and .bin output are created.
    bool raw_logging_enabled{false};
    /// Controls whether the PCD logger worker and .pcd output are created.
    bool pointcloud_logging_enabled{false};
    /// Delay before retrying while the selected LiDAR is unavailable.
    std::chrono::milliseconds lidar_reconnect_interval{5'000};
    application::GrafanaBridgeConfig grafana;
    application::SystemMonitorConfig system_monitor;
    std::optional<std::filesystem::path> raw_path;
    std::optional<std::filesystem::path> pcd_path;

    static AppConfig load();
    devices::LidarConfig lidar_config() const;
    RuntimeConfig runtime_config() const;
};

// Public for focused unit tests; normal application code should call AppConfig::load.
AppConfig parse_device_config_text(const std::string& contents);
std::string format_log_timestamp(
    std::chrono::system_clock::time_point timestamp);

}  // namespace vista
