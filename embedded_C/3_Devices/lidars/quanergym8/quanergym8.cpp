#include "3_Devices/lidars/quanergym8/quanergym8.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace vista::devices {

namespace {

constexpr std::uint8_t all_returns_packet_type = 0x00;
constexpr std::size_t firings_per_packet = 50;
constexpr std::size_t laser_count = 8;
constexpr std::size_t return_count = 3;
constexpr std::size_t all_returns_firing_size = 132;
constexpr std::size_t reduced_return_firing_size = 44;
constexpr double position_steps_per_rotation = 10'400.0;
constexpr double distance_unit_meters = 0.00001;
constexpr double tau = 6.283185307179586476925286766559;

constexpr std::array<double, laser_count> vertical_angles{
    -0.318505,
    -0.2692,
    -0.218009,
    -0.165195,
    -0.111003,
    -0.0557982,
    0.0,
    0.0557982,
};

std::uint16_t read_u16_be(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset) {
    if (offset + 2 > bytes.size()) {
        throw std::runtime_error("truncated M8 uint16 field");
    }
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes[offset]) << 8U) |
        static_cast<std::uint16_t>(bytes[offset + 1]));
}

std::uint32_t read_u32_be(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset) {
    if (offset + 4 > bytes.size()) {
        throw std::runtime_error("truncated M8 uint32 field");
    }
    return (static_cast<std::uint32_t>(bytes[offset]) << 24U) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 16U) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 8U) |
           static_cast<std::uint32_t>(bytes[offset + 3]);
}

std::uint32_t read_u32_be(
    const std::array<std::uint8_t, quanergy_m8_header_size>& bytes,
    std::size_t offset) {
    return (static_cast<std::uint32_t>(bytes.at(offset)) << 24U) |
           (static_cast<std::uint32_t>(bytes.at(offset + 1)) << 16U) |
           (static_cast<std::uint32_t>(bytes.at(offset + 2)) << 8U) |
           static_cast<std::uint32_t>(bytes.at(offset + 3));
}

std::uint64_t packet_timestamp_ns(const std::vector<std::uint8_t>& bytes) {
    const auto seconds = static_cast<std::uint64_t>(read_u32_be(bytes, 8));
    const auto nanoseconds = static_cast<std::uint64_t>(read_u32_be(bytes, 12));
    if (nanoseconds >= 1'000'000'000ULL) {
        throw std::runtime_error("invalid M8 packet nanosecond timestamp");
    }
    return seconds * 1'000'000'000ULL + nanoseconds;
}

std::uint8_t validate_common_header(const std::vector<std::uint8_t>& bytes) {
    if (bytes.size() < quanergy_m8_header_size) {
        throw std::runtime_error("truncated M8 packet header");
    }
    if (read_u32_be(bytes, 0) != quanergy_m8_packet_signature) {
        throw std::runtime_error("invalid M8 packet signature");
    }
    if (read_u32_be(bytes, 4) != bytes.size()) {
        throw std::runtime_error("M8 packet declared size does not match its data");
    }
    return bytes[19];
}

void push_point(
    std::vector<models::PointXYZIRT>& points,
    std::uint16_t position,
    std::size_t laser,
    std::uint8_t return_id,
    std::uint32_t raw_distance,
    std::uint8_t intensity,
    std::uint64_t timestamp_ns) {
    if (raw_distance == 0) {
        return;
    }

    const auto azimuth =
        static_cast<double>(position) * tau / position_steps_per_rotation;
    const auto vertical = vertical_angles.at(laser);
    const auto distance = static_cast<double>(raw_distance) * distance_unit_meters;
    const auto horizontal = distance * std::cos(vertical);

    points.push_back(models::PointXYZIRT{
        static_cast<float>(horizontal * std::cos(azimuth)),
        static_cast<float>(horizontal * std::sin(azimuth)),
        static_cast<float>(distance * std::sin(vertical)),
        intensity,
        static_cast<std::uint8_t>(laser),
        return_id,
        timestamp_ns,
    });
}

models::PointCloudFrame decode_all_returns(
    const std::vector<std::uint8_t>& bytes,
    std::uint64_t timestamp_ns) {
    if (bytes.size() != quanergy_m8_all_returns_packet_size) {
        throw std::runtime_error("incorrect M8 all-returns packet size");
    }

    const auto status_offset =
        quanergy_m8_header_size + firings_per_packet * all_returns_firing_size + 10;
    if (read_u16_be(bytes, status_offset) != 0) {
        throw std::runtime_error("M8 all-returns packet reports an error status");
    }

    std::vector<models::PointXYZIRT> points;
    points.reserve(firings_per_packet * laser_count * return_count);
    for (std::size_t firing = 0; firing < firings_per_packet; ++firing) {
        const auto base = quanergy_m8_header_size + firing * all_returns_firing_size;
        const auto position = read_u16_be(bytes, base);
        if (position >= 10'400) {
            throw std::runtime_error("invalid M8 firing position");
        }

        const auto distances_offset = base + 4;
        const auto intensities_offset =
            distances_offset + return_count * laser_count * 4;
        for (std::size_t return_id = 0; return_id < return_count; ++return_id) {
            for (std::size_t laser = 0; laser < laser_count; ++laser) {
                const auto sample = return_id * laser_count + laser;
                push_point(
                    points,
                    position,
                    laser,
                    static_cast<std::uint8_t>(return_id),
                    read_u32_be(bytes, distances_offset + sample * 4),
                    bytes.at(intensities_offset + sample),
                    timestamp_ns);
            }
        }
    }
    return {timestamp_ns, std::move(points)};
}

models::PointCloudFrame decode_reduced_return(
    const std::vector<std::uint8_t>& bytes,
    std::uint64_t timestamp_ns) {
    if (bytes.size() != quanergy_m8_reduced_return_packet_size) {
        throw std::runtime_error("incorrect M8 reduced-return packet size");
    }
    if (read_u16_be(bytes, quanergy_m8_header_size) != 0) {
        throw std::runtime_error("M8 reduced-return packet reports an error status");
    }

    const auto return_id = bytes.at(quanergy_m8_header_size + 2);
    if (return_id >= return_count) {
        throw std::runtime_error("invalid M8 return ID");
    }

    const auto firing_data_offset = quanergy_m8_header_size + 4;
    std::vector<models::PointXYZIRT> points;
    points.reserve(firings_per_packet * laser_count);
    for (std::size_t firing = 0; firing < firings_per_packet; ++firing) {
        const auto base = firing_data_offset + firing * reduced_return_firing_size;
        const auto position = read_u16_be(bytes, base);
        if (position >= 10'400) {
            throw std::runtime_error("invalid M8 firing position");
        }

        const auto distances_offset = base + 4;
        const auto intensities_offset = distances_offset + laser_count * 4;
        for (std::size_t laser = 0; laser < laser_count; ++laser) {
            push_point(
                points,
                position,
                laser,
                return_id,
                read_u32_be(bytes, distances_offset + laser * 4),
                bytes.at(intensities_offset + laser),
                timestamp_ns);
        }
    }
    return {timestamp_ns, std::move(points)};
}

}  // namespace

std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
QuanergyM8::connect(
    const std::string& ip,
    std::uint16_t port,
    std::chrono::milliseconds connection_timeout) {
    auto connection = transport::EthernetConnection::connect(
        ip,
        port,
        connection_timeout,
        connection_timeout);
    return {
        std::make_unique<QuanergyM8Reader>(std::move(connection)),
        std::make_unique<QuanergyM8Decoder>(),
    };
}

std::size_t QuanergyM8::validate_header(
    const std::array<std::uint8_t, quanergy_m8_header_size>& header) {
    if (read_u32_be(header, 0) != quanergy_m8_packet_signature) {
        throw std::runtime_error("invalid M8 packet signature");
    }

    const auto message_size = static_cast<std::size_t>(read_u32_be(header, 4));
    std::size_t expected_size{};
    switch (header[19]) {
        case all_returns_packet_type:
            expected_size = quanergy_m8_all_returns_packet_size;
            break;
        case quanergy_m8_reduced_return_type:
            expected_size = quanergy_m8_reduced_return_packet_size;
            break;
        default:
            throw std::runtime_error("unsupported M8 packet type");
    }
    if (message_size != expected_size) {
        throw std::runtime_error("M8 packet type declares an unexpected size");
    }
    return message_size;
}

RawPacket QuanergyM8Reader::read_raw_packet() {
    std::array<std::uint8_t, quanergy_m8_header_size> header{};
    connection_.read_exact(header.data(), header.size());
    const auto message_size = QuanergyM8::validate_header(header);

    std::vector<std::uint8_t> bytes(message_size);
    std::copy(header.begin(), header.end(), bytes.begin());
    connection_.read_exact(
        bytes.data() + quanergy_m8_header_size,
        bytes.size() - quanergy_m8_header_size);
    const auto timestamp = packet_timestamp_ns(bytes);
    return RawPacket(std::move(bytes), timestamp);
}

models::PointCloudFrame QuanergyM8Decoder::decode_packet(const RawPacket& packet) {
    const auto& bytes = packet.bytes();
    const auto packet_type = validate_common_header(bytes);
    const auto timestamp = packet_timestamp_ns(bytes);
    switch (packet_type) {
        case all_returns_packet_type:
            return decode_all_returns(bytes, timestamp);
        case quanergy_m8_reduced_return_type:
            return decode_reduced_return(bytes, timestamp);
        default:
            throw std::runtime_error("unsupported M8 packet type");
    }
}

}  // namespace vista::devices
