#pragma once

#include <cstdint>
#include <string>

#include "models/lidars/lidar_message.hpp"

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

using LidarGroundStatusMessage = LidarMessage<GroundStatus>;

}  // namespace vista::models
