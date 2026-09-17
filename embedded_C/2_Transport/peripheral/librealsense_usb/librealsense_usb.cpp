#include "2_Transport/peripheral/librealsense_usb/librealsense_usb.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#ifndef VISTA_HAS_REALSENSE
#define VISTA_HAS_REALSENSE 0
#endif

#if VISTA_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#endif

namespace vista::transport {

#if VISTA_HAS_REALSENSE

namespace {

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::uint64_t timestamp_ns(double timestamp_ms) {
    const auto value = timestamp_ms * 1'000'000.0;
    if (!std::isfinite(value) || value < 0.0 ||
        value > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
        throw std::runtime_error("invalid RealSense frame timestamp");
    }
    return static_cast<std::uint64_t>(std::llround(value));
}

bool same_intrinsics(const rs2_intrinsics& left, const rs2_intrinsics& right) {
    return left.width == right.width && left.height == right.height &&
           left.ppx == right.ppx && left.ppy == right.ppy &&
           left.fx == right.fx && left.fy == right.fy &&
           left.model == right.model &&
           std::equal(std::begin(left.coeffs), std::end(left.coeffs), std::begin(right.coeffs));
}

}  // namespace

struct LibrealsenseUsbConnection::Impl {
    rs2::context context;
    rs2::pipeline pipeline{context};
    std::optional<rs2::pipeline_profile> profile;
    std::chrono::milliseconds frame_timeout{};
    std::uint32_t width{};
    std::uint32_t height{};
    float depth_scale_m{};
    std::optional<rs2_intrinsics> intrinsics;
    std::vector<std::array<float, 3>> rays;
    bool calibration_locked{false};

    ~Impl() {
        if (profile) {
            try {
                pipeline.stop();
            } catch (...) {
                // Destructors must not propagate SDK shutdown failures.
            }
        }
    }

    void update_calibration(
        std::uint32_t new_width,
        std::uint32_t new_height,
        float new_depth_scale,
        const rs2_intrinsics& new_intrinsics) {
        if (!std::isfinite(new_depth_scale) || new_depth_scale <= 0.0F) {
            throw std::runtime_error("invalid RealSense depth scale");
        }
        if (new_intrinsics.width != static_cast<int>(new_width) ||
            new_intrinsics.height != static_cast<int>(new_height) ||
            !std::isfinite(new_intrinsics.fx) ||
            !std::isfinite(new_intrinsics.fy) ||
            new_intrinsics.fx <= 0.0F ||
            new_intrinsics.fy <= 0.0F) {
            throw std::runtime_error("invalid RealSense depth intrinsics");
        }
        if (new_intrinsics.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) {
            throw std::runtime_error(
                "modified Brown-Conrady depth deprojection is unsupported");
        }

        if (width == new_width && height == new_height &&
            depth_scale_m == new_depth_scale && intrinsics &&
            same_intrinsics(*intrinsics, new_intrinsics)) {
            return;
        }
        if (calibration_locked) {
            throw std::runtime_error(
                "RealSense calibration changed after decoder initialization");
        }

        std::vector<std::array<float, 3>> new_rays;
        new_rays.reserve(static_cast<std::size_t>(new_width) * new_height);
        for (std::uint32_t y = 0; y < new_height; ++y) {
            for (std::uint32_t x = 0; x < new_width; ++x) {
                const float pixel[2]{static_cast<float>(x), static_cast<float>(y)};
                std::array<float, 3> ray{};
                rs2_deproject_pixel_to_point(
                    ray.data(), &new_intrinsics, pixel, 1.0F);
                new_rays.push_back(ray);
            }
        }

        width = new_width;
        height = new_height;
        depth_scale_m = new_depth_scale;
        intrinsics = new_intrinsics;
        rays = std::move(new_rays);
    }
};

#else

struct LibrealsenseUsbConnection::Impl {};

#endif

LibrealsenseUsbConnection::LibrealsenseUsbConnection(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

LibrealsenseUsbConnection::LibrealsenseUsbConnection(
    LibrealsenseUsbConnection&&) noexcept = default;
LibrealsenseUsbConnection& LibrealsenseUsbConnection::operator=(
    LibrealsenseUsbConnection&&) noexcept = default;
LibrealsenseUsbConnection::~LibrealsenseUsbConnection() = default;

LibrealsenseUsbConnection LibrealsenseUsbConnection::open(
    const LibrealsenseDepthStreamConfig& config) {
#if VISTA_HAS_REALSENSE
    if (config.device_name_contains.empty() || config.width == 0 ||
        config.height == 0 || config.frames_per_second == 0 ||
        config.frame_timeout.count() <= 0) {
        throw std::invalid_argument("invalid RealSense depth-stream configuration");
    }

    try {
        auto implementation = std::make_unique<Impl>();
        const auto selector = lower_copy(config.device_name_contains);
        const auto requested_serial =
            config.serial_number && !config.serial_number->empty()
                ? config.serial_number
                : std::optional<std::string>{};
        std::string serial;
        std::string detected;

        for (const auto& device : implementation->context.query_devices()) {
            const auto name = device.supports(RS2_CAMERA_INFO_NAME)
                                  ? device.get_info(RS2_CAMERA_INFO_NAME)
                                  : "Unknown RealSense";
            const auto device_serial =
                device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                    ? std::optional<std::string>{
                          device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)}
                    : std::optional<std::string>{};
            if (!detected.empty()) {
                detected += ", ";
            }
            detected += name;
            if (device_serial) {
                detected += " [" + *device_serial + "]";
            }
            if (lower_copy(name).find(selector) != std::string::npos) {
                if (!device_serial) {
                    throw std::runtime_error(
                        "matching RealSense device has no serial number");
                }
                if (requested_serial && *device_serial != *requested_serial) {
                    continue;
                }
                serial = *device_serial;
                break;
            }
        }

        if (serial.empty()) {
            throw std::runtime_error(
                "RealSense device matching '" + config.device_name_contains +
                "'" +
                (requested_serial
                     ? " with USB serial '" + *requested_serial + "'"
                     : std::string{}) +
                " was not found (" +
                (detected.empty() ? "no RealSense devices detected" : detected) + ")");
        }

        rs2::config stream_config;
        stream_config.enable_device(serial);
        stream_config.enable_stream(
            RS2_STREAM_DEPTH,
            static_cast<int>(config.width),
            static_cast<int>(config.height),
            RS2_FORMAT_Z16,
            static_cast<int>(config.frames_per_second));
        implementation->profile = implementation->pipeline.start(stream_config);
        implementation->frame_timeout = config.frame_timeout;
        return LibrealsenseUsbConnection(std::move(implementation));
    } catch (const rs2::error& error) {
        throw std::runtime_error(std::string("librealsense: ") + error.what());
    }
#else
    (void)config;
    throw std::runtime_error(
        "RealSense backend is unavailable; configure librealsense2 and rebuild");
#endif
}

LibrealsenseDepthFrame LibrealsenseUsbConnection::read_depth_frame() {
#if VISTA_HAS_REALSENSE
    try {
        auto frames = implementation_->pipeline.wait_for_frames(
            static_cast<unsigned int>(implementation_->frame_timeout.count()));
        auto depth = frames.get_depth_frame();
        if (!depth) {
            throw std::runtime_error("RealSense frameset has no Z16 depth frame");
        }

        const auto width = static_cast<std::uint32_t>(depth.get_width());
        const auto height = static_cast<std::uint32_t>(depth.get_height());
        const auto stride = static_cast<std::size_t>(depth.get_stride_in_bytes());
        const auto bits = depth.get_bits_per_pixel();
        if (width == 0 || height == 0 || bits != 16) {
            throw std::runtime_error("unexpected RealSense depth-frame layout");
        }

        const auto packed_row_size = static_cast<std::size_t>(width) * 2U;
        if (stride < packed_row_size ||
            static_cast<std::size_t>(depth.get_data_size()) <
                stride * static_cast<std::size_t>(height)) {
            throw std::runtime_error("invalid RealSense depth-frame buffer size");
        }

        const auto video_profile = depth.get_profile().as<rs2::video_stream_profile>();
        implementation_->update_calibration(
            width,
            height,
            depth.get_units(),
            video_profile.get_intrinsics());

        const auto* source = static_cast<const std::uint8_t*>(depth.get_data());
        LibrealsenseDepthFrame output;
        output.timestamp_ns = timestamp_ns(depth.get_timestamp());
        output.bytes.reserve(packed_row_size * height);
        for (std::uint32_t row = 0; row < height; ++row) {
            const auto* begin = source + static_cast<std::size_t>(row) * stride;
            output.bytes.insert(output.bytes.end(), begin, begin + packed_row_size);
        }
        return output;
    } catch (const rs2::error& error) {
        throw std::runtime_error(std::string("librealsense: ") + error.what());
    }
#else
    throw std::runtime_error("RealSense backend is unavailable");
#endif
}

LibrealsenseDepthCalibration LibrealsenseUsbConnection::lock_calibration() {
#if VISTA_HAS_REALSENSE
    if (implementation_->rays.empty() || implementation_->depth_scale_m <= 0.0F) {
        throw std::runtime_error(
            "RealSense calibration is unavailable before the first frame");
    }
    implementation_->calibration_locked = true;
    return {implementation_->depth_scale_m, implementation_->rays};
#else
    throw std::runtime_error("RealSense backend is unavailable");
#endif
}

int LibrealsenseUsbConnection::runtime_api_version() noexcept {
#if VISTA_HAS_REALSENSE
    rs2_error* error = nullptr;
    const auto version = rs2_get_api_version(&error);
    if (error != nullptr) {
        rs2_free_error(error);
        return 0;
    }
    return version;
#else
    return 0;
#endif
}

}  // namespace vista::transport
