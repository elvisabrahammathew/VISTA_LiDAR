#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace vista::models {
enum class LocalizationState : std::uint8_t {
    waiting_imu, initializing, tracking, degraded, lost
};
struct LocalizationStatus {
    std::uint64_t timestamp_ns{}; // Host calendar time for monitoring; never used to propagate pose.
    LocalizationState state{LocalizationState::waiting_imu};
    std::array<double, 3> position_m{};
    std::array<double, 4> orientation_xyzw{0, 0, 0, 1};
    double match_ratio{}, residual_m{}, position_variance{};
    std::uint64_t local_map_points{}, rejected_scans{}, dropped_messages{};
    bool pose_valid{};
    std::string reason;
};
} // namespace vista::models
