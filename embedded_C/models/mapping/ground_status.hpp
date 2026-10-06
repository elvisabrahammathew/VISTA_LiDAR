#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace vista::models {

/// Lifecycle of the ground model used by point-cloud preprocessing.
enum class GroundState : std::uint8_t {
    calibrating = 0,
    valid = 1,
    static_fallback = 2,
    invalid = 3,
};

inline const char* to_string(GroundState state) noexcept {
    switch (state) {
        case GroundState::calibrating:
            return "calibrating";
        case GroundState::valid:
            return "valid";
        case GroundState::static_fallback:
            return "static_fallback";
        case GroundState::invalid:
            return "invalid";
    }
    return "invalid";
}

/// Lightweight diagnostics for validating one startup ground calibration.
struct GroundStatus {
    std::uint64_t timestamp_ns{};
    std::string configured_mode;
    GroundState state{GroundState::invalid};
    bool calibrated{};
    bool using_imu{};
    bool using_static_fallback{};

    float plane_a{};
    float plane_b{};
    float plane_c{1.0F};
    float plane_d{};
    float ground_tilt_deg{};
    float sensor_to_ground_distance_m{};
    float expected_ground_distance_m{};
    float ground_height_error_m{};

    std::uint64_t calibration_frame_count{};
    std::uint64_t calibration_sample_count{};
    std::uint64_t ground_inlier_count{};
    float ground_inlier_ratio{};
    float mean_residual_m{};
    float rms_residual_m{};
    float p95_residual_m{};

    std::uint64_t input_point_count{};
    std::uint64_t removed_ground_point_count{};
    std::uint64_t output_point_count{};
    float removed_ground_ratio{};
};

/// Ground diagnostics are a mapping topic, not a LiDAR transport envelope.
/// source_id identifies the point-cloud producer being calibrated.
struct GroundStatusMessage {
    std::string source_id;
    std::uint64_t sequence{};
    std::optional<std::uint64_t> sensor_timestamp_ns;
    std::uint64_t received_timestamp_ns{};
    GroundStatus payload;
    std::uint64_t received_monotonic_ns{};
    std::optional<std::uint64_t> measurement_timestamp_ns;

    GroundStatusMessage(std::string id, std::uint64_t message_sequence,
                        std::optional<std::uint64_t> sensor_timestamp,
                        std::uint64_t received_timestamp, GroundStatus value,
                        std::uint64_t received_monotonic = 0,
                        std::optional<std::uint64_t> measurement_timestamp = std::nullopt)
        : source_id(std::move(id)), sequence(message_sequence),
          sensor_timestamp_ns(sensor_timestamp), received_timestamp_ns(received_timestamp),
          payload(std::move(value)), received_monotonic_ns(received_monotonic),
          measurement_timestamp_ns(measurement_timestamp) {}
};

}  // namespace vista::models
