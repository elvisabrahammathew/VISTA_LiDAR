#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace vista {

namespace {

constexpr std::uint64_t default_frame_timeout_ms = 5'000;
constexpr std::uint64_t maximum_reconnect_interval_seconds = 86'400;
constexpr std::uint64_t maximum_grafana_interval_ms = 3'600'000;

#ifndef VISTA_DATA_ROOT
#define VISTA_DATA_ROOT "../data"
#endif

std::string trim(std::string value) {
    const auto first = std::find_if_not(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isspace(character) != 0;
        });
    const auto last = std::find_if_not(
        value.rbegin(), value.rend(), [](unsigned char character) {
            return std::isspace(character) != 0;
        }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string lower_copy(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

/// Removes a short device note such as "(quanergym8)" from a key or value.
std::string without_device_note(std::string value) {
    value = trim(std::move(value));
    const auto note = value.rfind('(');
    if (note != std::string::npos && !value.empty() && value.back() == ')') {
        value.erase(note);
    }
    return trim(std::move(value));
}

std::uint64_t parse_unsigned(
    const std::string& value,
    const std::string& label,
    std::uint64_t maximum) {
    std::size_t consumed = 0;
    std::uint64_t parsed{};
    try {
        parsed = std::stoull(value, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument("invalid " + label + " '" + value + "'");
    }
    if (consumed != value.size() || parsed == 0 || parsed > maximum) {
        throw std::invalid_argument(
            label + " must be between 1 and " + std::to_string(maximum));
    }
    return parsed;
}

bool parse_boolean(const std::string& value, const std::string& label) {
    const auto normalized = lower_copy(value);
    if (normalized == "true" || normalized == "yes" || normalized == "on" ||
        normalized == "1") {
        return true;
    }
    if (normalized == "false" || normalized == "no" || normalized == "off" ||
        normalized == "0") {
        return false;
    }
    throw std::invalid_argument(
        label + " must be true/false, yes/no, on/off, or 1/0");
}

bool parse_zero_one_switch(
    const std::string& value,
    const std::string& label) {
    if (value == "0") {
        return false;
    }
    if (value == "1") {
        return true;
    }
    throw std::invalid_argument(label + " must be 0 or 1");
}

void reject_duplicate(
    std::unordered_set<std::string>& keys,
    const std::string& key,
    std::size_t line_number) {
    if (!keys.insert(key).second) {
        throw std::invalid_argument(
            "duplicate device configuration key '" + key + "' at line " +
            std::to_string(line_number));
    }
}

}  // namespace

AppConfig parse_device_config_text(const std::string& contents) {
    AppConfig config;
    std::istringstream input(contents);
    std::unordered_set<std::string> keys;
    std::string original_line;
    std::size_t line_number = 0;
    bool lidar_seen = false;
    bool sensor_ip_seen = false;
    bool tcp_port_seen = false;
    bool usb_serial_seen = false;
    bool depth_width_seen = false;
    bool depth_height_seen = false;
    bool depth_fps_seen = false;

    while (std::getline(input, original_line)) {
        ++line_number;
        if (line_number == 1 && original_line.rfind("\xEF\xBB\xBF", 0) == 0) {
            original_line.erase(0, 3);
        }
        const auto comment = original_line.find('#');
        auto line = trim(original_line.substr(0, comment));
        if (line.empty()) {
            continue;
        }

        const auto separator = line.find(':');
        if (separator == std::string::npos) {
            throw std::invalid_argument(
                "invalid device configuration at line " +
                std::to_string(line_number) + ": expected 'Key: value'");
        }
        const auto key = lower_copy(
            without_device_note(line.substr(0, separator)));
        const auto value = without_device_note(line.substr(separator + 1));
        reject_duplicate(keys, key, line_number);

        if (key == "lidar") {
            if (value.empty() || lower_copy(value) == "none") {
                throw std::invalid_argument(
                    "DeviceConfig.txt must select quanergy-m8 or realsense-l515");
            }
            config.lidar_type = devices::parse_lidar_type(value);
            lidar_seen = true;
        } else if (key == "radar") {
            if (!value.empty() && lower_copy(value) != "none") {
                throw std::invalid_argument("Radar is not implemented yet");
            }
        } else if (key == "sensorip" || key == "ip") {
            if (value.empty()) {
                throw std::invalid_argument("SensorIP cannot be empty");
            }
            config.sensor_ip = value;
            sensor_ip_seen = true;
        } else if (key == "tcpport" || key == "port") {
            config.tcp_port = static_cast<std::uint16_t>(
                parse_unsigned(value, "TCP port", 65'535));
            tcp_port_seen = true;
        } else if (key == "usbserial" || key == "comport") {
            // L515 is a USB device. The former ComPort name is accepted as an
            // alias, but its value is interpreted as a librealsense serial.
            config.usb_serial = value.empty()
                                    ? std::optional<std::string>{}
                                    : std::optional<std::string>{value};
            usb_serial_seen = true;
        } else if (key == "depthwidth" || key == "width") {
            config.depth_width = static_cast<std::uint32_t>(parse_unsigned(
                value, "depth width", std::numeric_limits<std::uint32_t>::max()));
            depth_width_seen = true;
        } else if (key == "depthheight" || key == "height") {
            config.depth_height = static_cast<std::uint32_t>(parse_unsigned(
                value, "depth height", std::numeric_limits<std::uint32_t>::max()));
            depth_height_seen = true;
        } else if (key == "depthfps" || key == "fps") {
            config.depth_fps = static_cast<std::uint32_t>(parse_unsigned(
                value,
                "depth frame rate",
                std::numeric_limits<std::uint32_t>::max()));
            depth_fps_seen = true;
        } else if (key == "reconnectintervalseconds" ||
                   key == "lidarreconnectintervalseconds") {
            config.lidar_reconnect_interval = std::chrono::seconds(
                parse_unsigned(
                    value,
                     "LiDAR reconnect interval",
                     maximum_reconnect_interval_seconds));
        } else if (key == "rawloggingenabled") {
            config.raw_logging_enabled =
                parse_zero_one_switch(value, "RawLoggingEnabled(Lidar)");
        } else if (key == "pointcloudloggingenabled") {
            config.pointcloud_logging_enabled =
                parse_zero_one_switch(
                    value, "PointCloudLoggingEnabled(Lidar)");
        } else if (key == "grafanaenabled") {
            config.grafana.enabled = parse_boolean(value, "GrafanaEnabled");
        } else if (key == "grafanahost") {
            if (value.empty()) {
                throw std::invalid_argument("GrafanaHost cannot be empty");
            }
            config.grafana.host = value;
        } else if (key == "grafanaport") {
            config.grafana.port = static_cast<std::uint16_t>(
                parse_unsigned(value, "Grafana port", 65'535));
        } else if (key == "grafananamespace") {
            config.grafana.name_space = value;
        } else if (key == "grafanapublishintervalmilliseconds") {
            config.grafana.publish_interval = std::chrono::milliseconds(
                parse_unsigned(
                    value,
                    "Grafana publish interval",
                    maximum_grafana_interval_ms));
        } else if (key == "grafanaretryintervalseconds") {
            config.grafana.retry_interval = std::chrono::seconds(
                parse_unsigned(
                    value,
                    "Grafana retry interval",
                    maximum_reconnect_interval_seconds));
        } else if (key == "systemmonitorintervalmilliseconds") {
            config.system_monitor.sample_interval = std::chrono::milliseconds(
                parse_unsigned(
                    value,
                    "system monitor interval",
                    maximum_grafana_interval_ms));
        } else {
            throw std::invalid_argument(
                "unknown device key '" + key + "' at line " +
                std::to_string(line_number));
        }
    }

    if (!lidar_seen) {
        throw std::invalid_argument("DeviceConfig.txt is missing a Lidar selection");
    }
    if (config.lidar_type == devices::LidarType::quanergy_m8 &&
        (!sensor_ip_seen || !tcp_port_seen)) {
        throw std::invalid_argument(
            "Quanergy M8 requires SensorIP and TcpPort in DeviceConfig.txt");
    }
    if (config.lidar_type == devices::LidarType::realsense_l515 &&
        (!usb_serial_seen || !depth_width_seen || !depth_height_seen ||
         !depth_fps_seen)) {
        throw std::invalid_argument(
            "RealSense L515 requires UsbSerial, DepthWidth, DepthHeight, and "
            "DepthFps in DeviceConfig.txt");
    }
    application::validate_grafana_bridge_config(config.grafana);
    return config;
}

std::string format_log_timestamp(
    std::chrono::system_clock::time_point timestamp) {
    const auto raw_time = std::chrono::system_clock::to_time_t(timestamp);
    std::tm local_time{};
#ifdef _WIN32
    if (localtime_s(&local_time, &raw_time) != 0) {
        throw std::runtime_error("could not convert system time for log filename");
    }
#else
    if (localtime_r(&raw_time, &local_time) == nullptr) {
        throw std::runtime_error("could not convert system time for log filename");
    }
#endif

    std::ostringstream output;
    output << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return output.str();
}

AppConfig AppConfig::load() {
    std::ifstream input(default_device_config_path);
    if (!input) {
        throw std::runtime_error(
            "could not read device configuration '" +
            std::string(default_device_config_path) + "'");
    }
    std::ostringstream contents;
    contents << input.rdbuf();

    auto config = parse_device_config_text(contents.str());
    const auto data_root = std::filesystem::path(VISTA_DATA_ROOT).lexically_normal();
    config.system_monitor.data_root = data_root;
    if (config.raw_logging_enabled || config.pointcloud_logging_enabled) {
        const auto log_stem =
            format_log_timestamp(std::chrono::system_clock::now());
        if (config.raw_logging_enabled) {
            config.raw_path = data_root / "raw" / (log_stem + ".bin");
        }
        if (config.pointcloud_logging_enabled) {
            config.pcd_path =
                data_root / "processed" / (log_stem + ".pcd");
        }
    }
    return config;
}

devices::LidarConfig AppConfig::lidar_config() const {
    devices::LidarConfig config;
    config.lidar_type = lidar_type;
    config.sensor_ip = sensor_ip;
    config.port = tcp_port;
    // A connection attempt or an inactive data stream is considered lost
    // after five seconds. Reconnection uses the separately configured delay.
    config.connection_timeout =
        std::chrono::milliseconds(default_frame_timeout_ms);
    if (lidar_type == devices::LidarType::realsense_l515) {
        config.depth_stream = devices::DepthStreamConfig{
            usb_serial,
            depth_width,
            depth_height,
            depth_fps,
            config.connection_timeout,
        };
    }
    return config;
}

RuntimeConfig AppConfig::runtime_config() const {
    return {
        application::PreprocessingConfig{
            0.1F,
            200.0F,
            std::nullopt,
            0.05F,
            std::nullopt,
        },
        ThreadSetConfig{
            WorkerConfig{true, platform::ThreadConfig("lidar-read", 1)},
            WorkerConfig{true, platform::ThreadConfig("lidar-decode", 2)},
            WorkerConfig{
                raw_logging_enabled,
                platform::ThreadConfig("raw-logger", 2)},
            WorkerConfig{
                true, platform::ThreadConfig("pointcloud-preprocessing", 3)},
            WorkerConfig{
                pointcloud_logging_enabled,
                platform::ThreadConfig("pcd-logger", 2)},
            WorkerConfig{
                true, platform::ThreadConfig("system-monitor", 4)},
            WorkerConfig{
                grafana.enabled, platform::ThreadConfig("grafana-bridge", 4)},
        },
        TopicQueueConfig{32, 8, 8, 16},
    };
}

}  // namespace vista
