#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/lidars/ground_status.hpp"

namespace vista::application {

enum class GroundMode {
    static_height,
    ransac,
    hybrid,
};

const char* to_string(GroundMode mode) noexcept;
GroundMode parse_ground_mode(const std::string& value);

struct AxisAlignedRoi {
    float min_x{};
    float max_x{};
    float min_y{};
    float max_y{};
    float min_z{};
    float max_z{};
};

struct GroundRemovalConfig {
    GroundMode mode{GroundMode::static_height};
    float mount_x_m{};
    float mount_y_m{};
    float mount_z_m{};
    float mount_roll_deg{};
    float mount_pitch_deg{};
    float mount_yaw_deg{};
    float floor_z_m{};
    float distance_threshold_m{0.10F};
    float normal_tolerance_deg{15.0F};
    std::size_t calibration_frames{50};
    float minimum_inlier_ratio{0.20F};
    bool use_imu{false};
};

struct PreprocessingConfig {
    float min_distance_m{0.1F};
    float max_distance_m{200.0F};
    std::optional<AxisAlignedRoi> region_of_interest;
    std::optional<float> voxel_size_m{0.05F};
    std::optional<GroundRemovalConfig> ground_removal;
};

struct PreprocessingReport {
    std::uint64_t message_count{};
    std::uint64_t point_count{};
    std::uint64_t dropped_message_count{};
};

/// Stateful point-cloud processor used to calibrate one ground plane per run.
/// Mount pose is always the safe fallback when IMU data is unavailable.
class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(PreprocessingConfig config);
    ~PointCloudPreprocessor();

    PointCloudPreprocessor(const PointCloudPreprocessor&) = delete;
    PointCloudPreprocessor& operator=(const PointCloudPreprocessor&) = delete;
    PointCloudPreprocessor(PointCloudPreprocessor&&) noexcept;
    PointCloudPreprocessor& operator=(PointCloudPreprocessor&&) noexcept;

    /// Supplies the latest IMU sample. Invalid/non-stationary samples are ignored.
    void update_imu(const models::ImuFrame& sample);

    /// Converts sensor points to the common world frame, then filters them.
    models::PointCloudFrame process(models::PointCloudFrame frame);

    /// Static mode is ready immediately; RANSAC modes become ready after calibration.
    bool ground_calibrated() const noexcept;

    /// True only after a usable IMU gravity sample has actually been received.
    bool using_imu_orientation() const noexcept;

    /// Returns diagnostics generated while processing the most recent frame.
    models::GroundStatus ground_status() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

models::PointCloudFrame preprocess_point_cloud(
    models::PointCloudFrame frame,
    const PreprocessingConfig& config);

using PreprocessingCompletion =
    std::function<void(std::optional<PreprocessingReport>, std::string)>;

platform::WorkerHandle spawn_preprocessing_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    PreprocessingConfig config,
    PreprocessingCompletion on_complete);

}  // namespace vista::application
