#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <unordered_set>

#include "models/topics.hpp"

namespace vista::application {

namespace {

struct VoxelKey {
    int x{};
    int y{};
    int z{};

    bool operator==(const VoxelKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& value) const noexcept {
        auto seed = std::hash<int>{}(value.x);
        seed ^= std::hash<int>{}(value.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<int>{}(value.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

bool contains(const AxisAlignedRoi& roi, const models::PointXYZIRT& point) {
    return point.x >= roi.min_x && point.x <= roi.max_x &&
           point.y >= roi.min_y && point.y <= roi.max_y &&
           point.z >= roi.min_z && point.z <= roi.max_z;
}

void validate(const PreprocessingConfig& config) {
    if (!std::isfinite(config.min_distance_m) ||
        !std::isfinite(config.max_distance_m) ||
        config.min_distance_m < 0.0F ||
        config.max_distance_m < config.min_distance_m) {
        throw std::invalid_argument(
            "point-cloud distance limits must be finite, non-negative, and ordered");
    }

    if (config.region_of_interest) {
        const auto& roi = *config.region_of_interest;
        const float values[]{
            roi.min_x, roi.max_x, roi.min_y, roi.max_y, roi.min_z, roi.max_z};
        for (const auto value : values) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument("ROI bounds must be finite");
            }
        }
        if (roi.min_x > roi.max_x || roi.min_y > roi.max_y ||
            roi.min_z > roi.max_z) {
            throw std::invalid_argument("every ROI minimum must be <= its maximum");
        }
    }

    if (config.voxel_size_m &&
        (!std::isfinite(*config.voxel_size_m) || *config.voxel_size_m <= 0.0F)) {
        throw std::invalid_argument("voxel size must be positive and finite");
    }

    if (config.ground_removal &&
        (!std::isfinite(config.ground_removal->ground_height_m) ||
         !std::isfinite(config.ground_removal->tolerance_m) ||
         config.ground_removal->tolerance_m < 0.0F)) {
        throw std::invalid_argument(
            "ground height must be finite and tolerance non-negative");
    }
}

std::string current_exception_message() {
    try {
        throw;
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown preprocessing failure";
    }
}

}  // namespace

models::PointCloudFrame preprocess_point_cloud(
    models::PointCloudFrame frame,
    const PreprocessingConfig& config) {
    validate(config);

    const auto minimum_squared = config.min_distance_m * config.min_distance_m;
    const auto maximum_squared = config.max_distance_m * config.max_distance_m;
    frame.points.erase(
        std::remove_if(
            frame.points.begin(),
            frame.points.end(),
            [minimum_squared, maximum_squared](const auto& point) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) {
                    return true;
                }
                const auto distance_squared =
                    point.x * point.x + point.y * point.y + point.z * point.z;
                return distance_squared < minimum_squared ||
                       distance_squared > maximum_squared;
            }),
        frame.points.end());

    if (config.region_of_interest) {
        const auto roi = *config.region_of_interest;
        frame.points.erase(
            std::remove_if(
                frame.points.begin(),
                frame.points.end(),
                [roi](const auto& point) { return !contains(roi, point); }),
            frame.points.end());
    }

    if (config.voxel_size_m) {
        const auto voxel_size = *config.voxel_size_m;
        std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
        occupied.reserve(frame.points.size());
        frame.points.erase(
            std::remove_if(
                frame.points.begin(),
                frame.points.end(),
                [&occupied, voxel_size](const auto& point) {
                    const VoxelKey key{
                        static_cast<int>(std::floor(point.x / voxel_size)),
                        static_cast<int>(std::floor(point.y / voxel_size)),
                        static_cast<int>(std::floor(point.z / voxel_size)),
                    };
                    return !occupied.insert(key).second;
                }),
            frame.points.end());
    }

    if (config.ground_removal) {
        const auto ground = *config.ground_removal;
        frame.points.erase(
            std::remove_if(
                frame.points.begin(),
                frame.points.end(),
                [ground](const auto& point) {
                    return std::fabs(point.z - ground.ground_height_m) <=
                           ground.tolerance_m;
                }),
            frame.points.end());
    }

    return frame;
}

platform::WorkerHandle spawn_preprocessing_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    PreprocessingConfig config,
    PreprocessingCompletion on_complete) {
    // Preprocessing owns both its input and output topic endpoints.
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto decoded_input = inputs.subscribe<devices::LidarPointCloudMessage>(
        bus, models::topics::pointcloud_decoded);
    auto subscriber = std::move(decoded_input).take_subscriber();
    auto publisher = bus.publisher<devices::LidarPointCloudMessage>(
        models::topics::pointcloud_processed);
    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         subscriber = std::move(subscriber),
         publisher = std::move(publisher),
         config,
         on_complete = std::move(on_complete)]() mutable {
            try {
                PreprocessingReport report;
                std::shared_ptr<const devices::LidarPointCloudMessage> message;
                while (subscriber.receive(message) == platform::ReceiveStatus::message) {
                    auto processed = preprocess_point_cloud(message->payload, config);
                    ++report.message_count;
                    report.point_count += processed.points.size();
                    publisher.publish(devices::LidarPointCloudMessage(
                        message->lidar_id,
                        message->sequence,
                        message->sensor_timestamp_ns,
                        message->received_timestamp_ns,
                        std::move(processed)));
                }
                report.dropped_message_count = subscriber.dropped_messages();
                on_complete(report, {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
