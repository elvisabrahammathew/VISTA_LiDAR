#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "models/imu/imu.hpp"
#include "models/lidars/pointcloud.hpp"
#include "models/localization.hpp"

namespace vista::application {

struct LioConfig {
    bool enabled{false};
    // The single active IMU always uses lidar/imu and host monotonic timing.
    std::array<double, 3> initial_position_m{};
    std::array<double, 3> initial_rpy_deg{};
    // Transform from LiDAR coordinates into IMU coordinates; NOT initial world pose.
    std::array<double, 3> lidar_to_imu_translation_m{};
    std::array<double, 3> lidar_to_imu_rpy_deg{};
    double scan_voxel_m{0.15};
    double map_voxel_m{0.20};
    double local_radius_m{30.0};
    std::size_t local_max_points{50'000};
    std::size_t scan_max_points{5'000};
    std::size_t initialization_samples{50};
    double maximum_imu_gap_s{0.10};
    double minimum_match_ratio{0.15};
    double maximum_residual_m{0.15};
    double maximum_translation_step_m{2.0};
    double maximum_rotation_step_deg{30.0};
    std::chrono::milliseconds synchronization_timeout{500};
};

void validate_lio_config(const LioConfig& config);
struct TimedImu {
    double time_s{};
    std::array<double, 3> acceleration{};
    std::array<double, 3> angular_velocity{};
};
struct LioScan {
    models::PointCloudFrame cloud;
    double begin_s{}, end_s{};
    std::vector<TimedImu> imu; // Brackets the complete scan, including its end.
};
struct LioOutput {
    models::LocalizationStatus status;
    std::optional<models::PointCloudFrame> world_cloud;
};

// Owns the local odometry map only. The persistent RoomMap is a separate consumer.
class LioEngine {
public:
    explicit LioEngine(LioConfig config);
    ~LioEngine();
    LioEngine(const LioEngine&) = delete;
    LioEngine& operator=(const LioEngine&) = delete;
    LioOutput process(const LioScan& scan);
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

struct LioReport { std::uint64_t received{}, published{}, rejected{}, dropped{}; };
using LioCompletion = std::function<void(std::optional<LioReport>, std::string)>;
platform::WorkerHandle spawn_lio_worker(platform::MessageBus& bus,
    platform::ThreadConfig thread, platform::StopToken stop, LioConfig config,
    LioCompletion on_complete);

} // namespace vista::application
