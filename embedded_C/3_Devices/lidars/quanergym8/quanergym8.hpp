#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "2_Transport/peripheral/ethernet/ethernet.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::devices {

constexpr std::uint16_t quanergy_m8_default_port = 4141;
constexpr std::size_t quanergy_m8_header_size = 20;
constexpr std::size_t quanergy_m8_all_returns_packet_size = 6632;
constexpr std::size_t quanergy_m8_reduced_return_packet_size = 2224;
constexpr std::uint32_t quanergy_m8_packet_signature = 0x75bd7e97;
constexpr std::uint8_t quanergy_m8_reduced_return_type = 0x04;

class QuanergyM8Reader final : public ILidarReader {
public:
    explicit QuanergyM8Reader(transport::EthernetConnection connection)
        : connection_(std::move(connection)) {}

    RawPacket read_raw_packet() override;

private:
    transport::EthernetConnection connection_;
};

class QuanergyM8Decoder final : public ILidarDecoder {
public:
    models::PointCloudFrame decode_packet(const RawPacket& packet) override;
};

struct QuanergyM8 {
    static std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
    connect(
        const std::string& ip,
        std::uint16_t port,
        std::chrono::milliseconds connection_timeout);

    static std::size_t validate_header(
        const std::array<std::uint8_t, quanergy_m8_header_size>& header);
};

}  // namespace vista::devices
