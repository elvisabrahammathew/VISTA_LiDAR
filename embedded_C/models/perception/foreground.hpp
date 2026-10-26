#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "models/mapping/room_map.hpp"

namespace vista::models {

struct ForegroundPoint { float x{}, y{}, z{}; std::uint8_t intensity{}; };

// Worker-local bounded query, not a mapper-produced topic. Background is the
// newest completed checkpoint causally older than the selected scan receipt.
struct ForegroundObservation {
    std::string source;
    std::uint64_t sequence{}, timestamp_ns{}, monotonic_ns{}, received_monotonic_ns{}, epoch{};
    std::shared_ptr<const RoomMapSurfaceSnapshot> background;
    std::vector<ForegroundPoint> points;
};

struct ForegroundFrame {
    std::string source;
    std::uint64_t sequence{}, timestamp_ns{}, monotonic_ns{}, received_monotonic_ns{}, epoch{}, background_revision{};
    std::vector<ForegroundPoint> points;
    std::size_t input_points{}, background_points{}, unknown_points{}, cache_points{}, cache_regions{}, region_reads{}, distance_checks{};
    std::string reason; // Foreground is NOT a motion/semantic classification.
};

} // namespace vista::models
