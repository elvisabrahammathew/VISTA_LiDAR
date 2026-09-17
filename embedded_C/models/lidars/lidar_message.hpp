#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace vista::models {

/// LiDAR-only topic envelope with ordering and timing metadata.
template <typename T>
struct LidarMessage {
    std::string lidar_id;
    std::uint64_t sequence{};
    std::optional<std::uint64_t> sensor_timestamp_ns;
    std::uint64_t received_timestamp_ns{};
    T payload;

    LidarMessage(
        std::string id,
        std::uint64_t message_sequence,
        std::optional<std::uint64_t> sensor_timestamp,
        std::uint64_t received_timestamp,
        T value)
        : lidar_id(std::move(id)),
          sequence(message_sequence),
          sensor_timestamp_ns(sensor_timestamp),
          received_timestamp_ns(received_timestamp),
          payload(std::move(value)) {}
};

}  // namespace vista::models
