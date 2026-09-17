#pragma once

#include <array>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "2_Transport/peripheral/librealsense_usb/librealsense_usb.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::devices {

class RealSenseL515Reader final : public ILidarReader {
public:
    RealSenseL515Reader(
        transport::LibrealsenseUsbConnection connection,
        RawPacket pending_packet);

    RawPacket read_raw_packet() override;

private:
    transport::LibrealsenseUsbConnection connection_;
    std::optional<RawPacket> pending_packet_;
};

class RealSenseL515Decoder final : public ILidarDecoder {
public:
    RealSenseL515Decoder(
        float depth_scale_m,
        std::vector<std::array<float, 3>> rays);

    models::PointCloudFrame decode_packet(const RawPacket& packet) override;

private:
    float depth_scale_m_;
    std::vector<std::array<float, 3>> rays_;
};

struct RealSenseL515 {
    static std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
    connect(const DepthStreamConfig& config);
};

models::PointCloudFrame decode_l515_depth_frame(
    const std::vector<std::uint8_t>& bytes,
    const std::vector<std::array<float, 3>>& rays,
    float depth_scale_m,
    std::uint64_t timestamp_ns);

}  // namespace vista::devices
