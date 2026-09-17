#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/pubsub/pubsub.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::application {

struct LoggerReport {
    std::uint64_t message_count{};
    std::uint64_t point_count{};
    std::uint64_t dropped_message_count{};
};

using LoggerCompletion =
    std::function<void(std::optional<LoggerReport>, std::string)>;

platform::WorkerHandle spawn_raw_logger(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::filesystem::path path,
    LoggerCompletion on_complete);

platform::WorkerHandle spawn_pcd_logger(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::filesystem::path path,
    LoggerCompletion on_complete);

LoggerReport run_raw_logger(
    platform::TopicSubscriber<devices::LidarRawMessage> subscriber,
    const std::filesystem::path& path);

LoggerReport run_pcd_logger(
    platform::TopicSubscriber<devices::LidarPointCloudMessage> subscriber,
    const std::filesystem::path& path);

}  // namespace vista::application
