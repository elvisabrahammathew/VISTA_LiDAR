#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace vista::models {

/// Sensor-neutral point using meters and a nanosecond source timestamp.
struct PointXYZIRT {
    float x{};
    float y{};
    float z{};
    std::uint8_t intensity{};
    std::uint8_t ring{};
    std::uint8_t return_id{};
    std::uint64_t timestamp_ns{};

    bool operator==(const PointXYZIRT& other) const {
        return x == other.x && y == other.y && z == other.z &&
               intensity == other.intensity && ring == other.ring &&
               return_id == other.return_id && timestamp_ns == other.timestamp_ns;
    }
};

/// Groups all valid points decoded from one sensor packet or depth frame.
struct PointCloudFrame {
    std::uint64_t timestamp_ns{};
    std::vector<PointXYZIRT> points;
    /// Valid only after mounting: ray origins and endpoints share world coordinates.
    /// Missing origin disables free-space clearing rather than guessing an origin.
    std::optional<std::array<float, 3>> sensor_origin_world_m;

    PointCloudFrame() = default;
    PointCloudFrame(std::uint64_t timestamp, std::vector<PointXYZIRT> values,
                    std::optional<std::array<float, 3>> origin = std::nullopt)
        : timestamp_ns(timestamp), points(std::move(values)), sensor_origin_world_m(origin) {}
};

}  // namespace vista::models
