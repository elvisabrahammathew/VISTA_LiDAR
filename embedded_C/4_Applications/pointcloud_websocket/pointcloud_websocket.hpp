#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/lidars/ground_status.hpp"

namespace vista::application {

struct PointCloudWebSocketConfig {
    bool enabled{false};
    std::string bind_address{"127.0.0.1"};
    std::uint16_t port{8765};
    std::size_t maximum_points{100'000};
    std::size_t maximum_clients{4};
    std::chrono::milliseconds publish_interval{100};
    std::chrono::milliseconds retry_interval{2'000};
};

void validate_pointcloud_websocket_config(
    const PointCloudWebSocketConfig& config);

/// Encodes a processed cloud as an LPC1 little-endian XYZ+intensity packet.
/// Large frames are sampled uniformly to keep browser/network load bounded.
std::vector<std::uint8_t> encode_lpc1_pointcloud(
    const devices::LidarPointCloudMessage& message,
    std::size_t maximum_points,
    const models::GroundStatus* ground_status = nullptr);

struct PointCloudWebSocketReport {
    std::uint64_t received_messages{};
    std::uint64_t broadcast_frames{};
    std::uint64_t client_deliveries{};
    std::uint64_t encoded_points{};
    std::uint64_t dropped_input_messages{};
    std::uint64_t server_failures{};
};

using PointCloudWebSocketCompletion = std::function<void(
    std::optional<PointCloudWebSocketReport>,
    std::string)>;

/// Starts an independent worker that subscribes to pointcloud/processed and
/// broadcasts LPC1 frames to every connected Grafana browser panel.
platform::WorkerHandle spawn_pointcloud_websocket(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    PointCloudWebSocketConfig config,
    PointCloudWebSocketCompletion on_complete);

}  // namespace vista::application
