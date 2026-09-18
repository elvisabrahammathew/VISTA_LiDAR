#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::application {

inline constexpr const char* grafana_token_environment_variable =
    "VISTA_GRAFANA_TOKEN";
inline constexpr const char* grafana_secret_file_name = "GrafanaSecret.txt";

struct GrafanaBridgeConfig {
    bool enabled{true};
    std::string host{"127.0.0.1"};
    std::uint16_t port{3000};
    std::string name_space{"vista"};
    std::chrono::milliseconds publish_interval{1'000};
    std::chrono::milliseconds retry_interval{5'000};
    std::chrono::milliseconds request_timeout{2'000};
    std::chrono::milliseconds offline_timeout{5'000};
};

/// Validates endpoint and timing values before any worker is started.
void validate_grafana_bridge_config(const GrafanaBridgeConfig& config);

/// Builds the Authorization header without exposing the token in logs or the
/// text configuration file.
std::string make_grafana_bearer_authorization(const std::string& token);

struct PointCloudTelemetry {
    std::string lidar_id;
    std::uint64_t timestamp_ns{};
    std::uint64_t input_point_count{};
    std::uint64_t point_count{};
    std::uint64_t dropped_messages{};
    double frames_per_second{};
    double processing_ms{};
    bool online{};
    bool has_bounds{};
    float min_x_m{};
    float max_x_m{};
    float min_y_m{};
    float max_y_m{};
    float min_z_m{};
    float max_z_m{};
};

/// Converts one telemetry sample to the Influx line protocol accepted by
/// Grafana Live's HTTP push endpoint.
std::string format_pointcloud_measurement(
    const PointCloudTelemetry& telemetry);

struct GrafanaBridgeReport {
    std::uint64_t received_raw_messages{};
    std::uint64_t received_decoded_messages{};
    std::uint64_t received_processed_messages{};
    std::uint64_t published_measurements{};
    std::uint64_t failed_publish_attempts{};
    std::uint64_t dropped_input_messages{};
};

using GrafanaBridgeCompletion =
    std::function<void(std::optional<GrafanaBridgeReport>, std::string)>;

/// Starts a worker that owns all Grafana-related subscriptions. More sensor
/// and application topics can be added here without changing main.cpp.
platform::WorkerHandle spawn_grafana_bridge(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    GrafanaBridgeConfig config,
    GrafanaBridgeCompletion on_complete);

}  // namespace vista::application
