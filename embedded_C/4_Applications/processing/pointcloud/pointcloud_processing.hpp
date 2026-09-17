#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::application {

struct AxisAlignedRoi {
    float min_x{};
    float max_x{};
    float min_y{};
    float max_y{};
    float min_z{};
    float max_z{};
};

struct GroundRemovalConfig {
    float ground_height_m{};
    float tolerance_m{};
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
