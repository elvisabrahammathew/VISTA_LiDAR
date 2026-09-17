#pragma once

#include <cstdint>
#include <vector>

namespace vista::models {

// Generic image container reserved for future camera-based applications.
struct ImageFrame {
    std::uint64_t timestamp_ns{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<std::uint8_t> bytes;
};

}  // namespace vista::models
