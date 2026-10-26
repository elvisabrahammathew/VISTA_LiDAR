#pragma once

namespace vista::application {

/// Initial LiDAR world pose for LIO, or the fixed sensor-to-world transform
/// without LIO. Independent of floor detection and LiDAR-to-IMU calibration.
struct MountingConfig {
    float mount_x_m{};
    float mount_y_m{};
    float mount_z_m{};
    float mount_roll_deg{};
    float mount_pitch_deg{};
    float mount_yaw_deg{};
};

} // namespace vista::application
