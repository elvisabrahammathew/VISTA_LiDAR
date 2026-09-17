#include "3_Devices/lidars/realsensel515/realsensel515.hpp"

#include <cmath>
#include <stdexcept>

namespace vista::devices {

namespace {

RawPacket raw_packet_from_frame(transport::LibrealsenseDepthFrame frame) {
    return RawPacket(std::move(frame.bytes), frame.timestamp_ns);
}

}  // namespace

RealSenseL515Reader::RealSenseL515Reader(
    transport::LibrealsenseUsbConnection connection,
    RawPacket pending_packet)
    : connection_(std::move(connection)),
      pending_packet_(std::move(pending_packet)) {}

RawPacket RealSenseL515Reader::read_raw_packet() {
    if (pending_packet_) {
        auto packet = std::move(*pending_packet_);
        pending_packet_.reset();
        return packet;
    }
    return raw_packet_from_frame(connection_.read_depth_frame());
}

RealSenseL515Decoder::RealSenseL515Decoder(
    float depth_scale_m,
    std::vector<std::array<float, 3>> rays)
    : depth_scale_m_(depth_scale_m), rays_(std::move(rays)) {}

models::PointCloudFrame RealSenseL515Decoder::decode_packet(
    const RawPacket& packet) {
    return decode_l515_depth_frame(
        packet.bytes(),
        rays_,
        depth_scale_m_,
        packet.timestamp_ns().value_or(0));
}

std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
RealSenseL515::connect(const DepthStreamConfig& config) {
    transport::LibrealsenseDepthStreamConfig transport_config{
        "L515",
        config.usb_serial,
        config.width,
        config.height,
        config.frames_per_second,
        config.frame_timeout,
    };
    auto connection = transport::LibrealsenseUsbConnection::open(transport_config);

    // The first frame initializes calibration and is kept for sequence zero.
    auto first_frame = connection.read_depth_frame();
    auto calibration = connection.lock_calibration();
    auto first_packet = raw_packet_from_frame(std::move(first_frame));

    return {
        std::make_unique<RealSenseL515Reader>(
            std::move(connection), std::move(first_packet)),
        std::make_unique<RealSenseL515Decoder>(
            calibration.depth_scale_m, std::move(calibration.rays)),
    };
}

models::PointCloudFrame decode_l515_depth_frame(
    const std::vector<std::uint8_t>& bytes,
    const std::vector<std::array<float, 3>>& rays,
    float depth_scale_m,
    std::uint64_t timestamp_ns) {
    if (bytes.size() != rays.size() * 2U) {
        throw std::runtime_error("L515 depth buffer has an unexpected size");
    }
    if (!std::isfinite(depth_scale_m) || depth_scale_m <= 0.0F) {
        throw std::runtime_error("L515 depth scale must be positive and finite");
    }

    std::vector<models::PointXYZIRT> points;
    points.reserve(rays.size());
    for (std::size_t index = 0; index < rays.size(); ++index) {
        const auto byte_index = index * 2U;
        const auto raw_depth = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[byte_index]) |
            (static_cast<std::uint16_t>(bytes[byte_index + 1]) << 8U));
        if (raw_depth == 0) {
            continue;
        }

        const auto depth_m = static_cast<float>(raw_depth) * depth_scale_m;
        const auto& ray = rays[index];
        points.push_back(models::PointXYZIRT{
            ray[2] * depth_m,
            -ray[0] * depth_m,
            -ray[1] * depth_m,
            0,
            0,
            0,
            timestamp_ns,
        });
    }
    return {timestamp_ns, std::move(points)};
}

}  // namespace vista::devices
