#pragma once
#include "models/mapping/localization.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/mapping/ground_status.hpp"
#include "models/mapping/room_map.hpp"
#include "models/telemetry.hpp"

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
    /// Read from the selected LiDAR configuration, never a temporary device tag.
    std::string selected_lidar_id{"quanergy-m8"};
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
/// Stable selected-LiDAR channel: tag-free fields prevent startup ghost series.
std::string format_current_pointcloud_measurement(const PointCloudTelemetry& telemetry);
PointCloudTelemetry make_offline_pointcloud_telemetry(const std::string& lidar_id);

struct ImuTelemetry {
    std::string lidar_id;
    std::uint64_t timestamp_ns{};
    double sample_rate_hz{};
    bool online{};
    models::ImuFrame sample;
};

/// Converts one IMU sample to the dedicated Grafana Live measurement.
std::string format_imu_measurement(const ImuTelemetry& telemetry);

/// Publishes the five ground-quality values shown by Ground Validation.
/// Plane/IMU flags remain available internally and in the 3D WebSocket header.
std::string format_ground_measurement(
    const models::GroundStatusMessage& message);

/// Publishes the current state on a tag-free measurement so Grafana Stat
/// always receives exactly one stable series.
std::string format_ground_state_measurement(
    const models::GroundStatusMessage& message);

/// Tag-free mapping diagnostics for stable Grafana Stat fields.
std::string format_room_map_measurement(const models::RoomMapStatus& status);
std::string format_localization_measurement(const models::LocalizationStatus& status);

/// Stable table rows: worker is an ordinary string column, never a series tag.
/// Distinct millisecond timestamps prevent Grafana Live from joining workers
/// into the same row. Internal MessageBus worker telemetry is unchanged.
std::vector<std::string> format_worker_health_measurements(
    std::vector<models::WorkerHealthTelemetry> workers,
    std::uint64_t publication_timestamp_ns);

struct GrafanaBridgeReport {
    std::uint64_t received_raw_messages{};
    std::uint64_t received_decoded_messages{};
    std::uint64_t received_processed_messages{};
    std::uint64_t received_imu_messages{};
    std::uint64_t received_ground_messages{};
    std::uint64_t published_measurements{};
    std::uint64_t failed_publish_attempts{};
    std::uint64_t dropped_input_messages{};
};

using GrafanaBridgeCompletion =
    std::function<void(std::optional<GrafanaBridgeReport>, std::string)>;

/// Starts a worker with compact measurements matching dashboard UID vistalive.
/// It does not publish the legacy pointcloud, storage or power channels.
/// More sensor
/// and application topics can be added here without changing main.cpp.
platform::WorkerHandle spawn_grafana_bridge(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    GrafanaBridgeConfig config,
    GrafanaBridgeCompletion on_complete);

}  // namespace vista::application
