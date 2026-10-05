#pragma once

#include <cstdint>
#include <string>
#include "models/lidars/lidar_message.hpp"

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

/// Common lidar/imu envelope. lidar_id is the originating sensor ID (legacy
/// name), not a source-selection config. External drivers must populate host
/// monotonic timing using platform::MeasurementClock at acquisition, not decode.
using ImuMessage = LidarMessage<ImuFrame>;

}  // namespace vista::models
