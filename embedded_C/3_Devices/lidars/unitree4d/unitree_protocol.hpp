#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "models/lidars/pointcloud.hpp"

namespace vista::devices::unitree_protocol {

// Values follow Unitree's public unilidar_sdk2 protocol headers. The project
// parses the wire format itself so the same code runs on every target OS.
inline constexpr std::size_t frame_header_size = 12;
inline constexpr std::size_t frame_tail_size = 12;
inline constexpr std::size_t maximum_frame_size = 65'536;
inline constexpr std::uint32_t point_packet_type = 102;
inline constexpr std::uint32_t point_2d_packet_type = 103;
inline constexpr std::uint32_t imu_packet_type = 104;
inline constexpr std::uint32_t version_packet_type = 105;
inline constexpr std::size_t points_per_packet = 300;
inline constexpr std::size_t default_cloud_scan_count = 18;

std::uint32_t read_u32_le(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset);
float read_f32_le(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset);
std::uint32_t crc32(const std::uint8_t* data, std::size_t size);
std::uint32_t packet_type(const std::vector<std::uint8_t>& frame);
std::optional<std::uint64_t> packet_timestamp_ns(
    const std::vector<std::uint8_t>& frame);
void validate_frame(const std::vector<std::uint8_t>& frame);

/// Decodes one 3D range packet; non-point packet types are handled elsewhere.
std::vector<models::PointXYZIRT> decode_point_packet(
    const std::vector<std::uint8_t>& frame,
    std::uint8_t ring);

}  // namespace vista::devices::unitree_protocol
