#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "3_Devices/lidars/lidar.hpp"

namespace vista::devices {

/// Reads validated Unitree wire frames from serial/USB or Ethernet/UDP.
class UnitreeL2Reader final : public ILidarReader {
public:
    struct Impl;

    explicit UnitreeL2Reader(std::unique_ptr<Impl> implementation);
    ~UnitreeL2Reader() override;
    RawPacket read_raw_packet() override;
    void keep_for_first_read(RawPacket packet);

private:
    std::unique_ptr<Impl> implementation_;
    std::optional<RawPacket> pending_packet_;
};

/// Builds one sensor-neutral cloud from the configured number of L2 scan rows.
class UnitreeL2Decoder final : public ILidarDecoder {
public:
    explicit UnitreeL2Decoder(std::size_t cloud_scan_count = 18);
    models::PointCloudFrame decode_packet(const RawPacket& packet) override;

private:
    std::size_t cloud_scan_count_;
    std::size_t accumulated_scan_count_{};
    std::uint64_t cloud_timestamp_ns_{};
    std::vector<models::PointXYZIRT> accumulated_points_;
};

struct UnitreeL2 {
    static std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
    connect(
        const UnitreeL2Config& config,
        std::chrono::milliseconds connection_timeout);
};

}  // namespace vista::devices
