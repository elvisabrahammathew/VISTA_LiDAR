#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace vista::models {

/// One IMU sample from the single active integrated OR external motion sensor.
/// Angular velocity uses radians/second and acceleration uses metres/second^2.
struct ImuFrame {
    std::uint64_t timestamp_ns{};
    float orientation_x{};
    float orientation_y{};
    float orientation_z{};
    float orientation_w{1.0F};
    float angular_velocity_x_rad_s{};
    float angular_velocity_y_rad_s{};
    float angular_velocity_z_rad_s{};
    float linear_acceleration_x_m_s2{};
    float linear_acceleration_y_m_s2{};
    float linear_acceleration_z_m_s2{};
};

/// Independent envelope for the one active integrated OR external IMU.
/// Host monotonic timing is captured/aligned at acquisition, before queues.
/// Device time only preserves sample intervals and detects resets; calendar
/// received_timestamp_ns is for monitoring, never pose propagation.
struct ImuMessage {
    std::string source_id;
    std::uint64_t sequence{};
    std::optional<std::uint64_t> sensor_timestamp_ns;
    std::uint64_t received_timestamp_ns{};
    ImuFrame payload;
    std::uint64_t received_monotonic_ns{};
    std::optional<std::uint64_t> measurement_timestamp_ns;

    ImuMessage(std::string id, std::uint64_t message_sequence,
               std::optional<std::uint64_t> sensor_timestamp,
               std::uint64_t received_timestamp, ImuFrame value,
               std::uint64_t received_monotonic = 0,
               std::optional<std::uint64_t> measurement_timestamp = std::nullopt)
        : source_id(std::move(id)), sequence(message_sequence),
          sensor_timestamp_ns(sensor_timestamp), received_timestamp_ns(received_timestamp),
          payload(std::move(value)), received_monotonic_ns(received_monotonic),
          measurement_timestamp_ns(measurement_timestamp) {}
};

}  // namespace vista::models
