#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vista::transport {

struct LibrealsenseDepthStreamConfig {
    std::string device_name_contains;
    /// Optional librealsense serial used when more than one device is present.
    std::optional<std::string> serial_number;
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t frames_per_second{};
    std::chrono::milliseconds frame_timeout{};
};

struct LibrealsenseDepthFrame {
    std::vector<std::uint8_t> bytes;
    std::uint64_t timestamp_ns{};
};

struct LibrealsenseDepthCalibration {
    float depth_scale_m{};
    std::vector<std::array<float, 3>> rays;
};

/// Owns librealsense context/pipeline/frame access and exposes only safe values.
class LibrealsenseUsbConnection {
public:
    static LibrealsenseUsbConnection open(const LibrealsenseDepthStreamConfig& config);

    LibrealsenseUsbConnection(const LibrealsenseUsbConnection&) = delete;
    LibrealsenseUsbConnection& operator=(const LibrealsenseUsbConnection&) = delete;
    LibrealsenseUsbConnection(LibrealsenseUsbConnection&&) noexcept;
    LibrealsenseUsbConnection& operator=(LibrealsenseUsbConnection&&) noexcept;
    ~LibrealsenseUsbConnection();

    LibrealsenseDepthFrame read_depth_frame();
    LibrealsenseDepthCalibration lock_calibration();
    static int runtime_api_version() noexcept;

private:
    struct Impl;
    explicit LibrealsenseUsbConnection(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> implementation_;
};

}  // namespace vista::transport
