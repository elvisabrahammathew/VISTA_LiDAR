#pragma once

#include <cstdint>

namespace vista::models {

/// One IMU sample decoded from a LiDAR-integrated motion sensor.
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

}  // namespace vista::models
