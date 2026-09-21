#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "3_Devices/lidars/unitree4d/unitree4d.hpp"
#include "3_Devices/lidars/unitree4d/unitree_protocol.hpp"
#include "unittest/test.hpp"

namespace {

void write_u16_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint32_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
    bytes[offset + 2] = static_cast<std::uint8_t>(value >> 16U);
    bytes[offset + 3] = static_cast<std::uint8_t>(value >> 24U);
}

void write_f32_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    float value) {
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_le(bytes, offset, bits);
}

vista::devices::RawPacket one_meter_unitree_packet() {
    constexpr std::size_t packet_size = 1'044;
    constexpr std::size_t calibration_offset = 64;
    constexpr std::size_t line_offset = 96;
    constexpr std::size_t point_count_offset = 128;
    constexpr std::size_t ranges_offset = 132;
    constexpr std::size_t intensities_offset = 732;
    constexpr std::size_t tail_offset = packet_size - 12;

    std::vector<std::uint8_t> bytes(packet_size);
    bytes[0] = 0x55;
    bytes[1] = 0xAA;
    bytes[2] = 0x05;
    bytes[3] = 0x0A;
    write_u32_le(bytes, 4, vista::devices::unitree_protocol::point_packet_type);
    write_u32_le(bytes, 8, packet_size);
    write_u32_le(bytes, 12, 7);  // sequence
    write_u32_le(bytes, 16, packet_size - 24);
    write_u32_le(bytes, 20, 1);  // timestamp seconds
    write_u32_le(bytes, 24, 2);  // timestamp nanoseconds

    // xi=-pi/2 and zero alpha/theta produce one point on positive X.
    write_f32_le(bytes, calibration_offset + 20, -1.57079632679F);
    write_f32_le(bytes, calibration_offset + 28, 0.001F);
    write_f32_le(bytes, line_offset + 12, 0.0F);
    write_f32_le(bytes, line_offset + 16, 100.0F);
    write_f32_le(bytes, line_offset + 28, 0.00001F);
    write_u32_le(bytes, point_count_offset, 1);
    write_u16_le(bytes, ranges_offset, 1'000);
    bytes[intensities_offset] = 42;
    bytes[packet_size - 2] = 0x00;
    bytes[packet_size - 1] = 0xFF;
    write_u32_le(
        bytes,
        tail_offset,
        vista::devices::unitree_protocol::crc32(
            bytes.data() + vista::devices::unitree_protocol::frame_header_size,
            packet_size -
                vista::devices::unitree_protocol::frame_header_size -
                vista::devices::unitree_protocol::frame_tail_size));
    return vista::devices::RawPacket(std::move(bytes), 1'000'000'002ULL);
}

}  // namespace

VISTA_TEST(unitree_validates_and_decodes_one_meter_point) {
    const auto packet = one_meter_unitree_packet();
    vista::devices::unitree_protocol::validate_frame(packet.bytes());
    const auto points = vista::devices::unitree_protocol::decode_point_packet(
        packet.bytes(), 3);

    VISTA_CHECK(points.size() == 1);
    VISTA_CHECK_NEAR(points[0].x, 1.0F, 1.0e-5F);
    VISTA_CHECK_NEAR(points[0].y, 0.0F, 1.0e-5F);
    VISTA_CHECK_NEAR(points[0].z, 0.0F, 1.0e-5F);
    VISTA_CHECK(points[0].intensity == 42);
    VISTA_CHECK(points[0].ring == 3);
    VISTA_CHECK(points[0].timestamp_ns == 1'000'000'002ULL);
}

VISTA_TEST(unitree_decoder_emits_cloud_after_configured_scan_count) {
    vista::devices::UnitreeL2Decoder decoder(2);
    auto first = decoder.decode_packet(one_meter_unitree_packet());
    VISTA_CHECK(first.points.empty());

    auto second = decoder.decode_packet(one_meter_unitree_packet());
    VISTA_CHECK(second.points.size() == 2);
    VISTA_CHECK(second.points[0].ring == 0);
    VISTA_CHECK(second.points[1].ring == 1);
    VISTA_CHECK(second.timestamp_ns == 1'000'000'002ULL);
}

VISTA_TEST(unitree_rejects_corrupted_crc) {
    auto packet = one_meter_unitree_packet();
    auto bytes = packet.bytes();
    bytes[200] ^= 0x01;
    VISTA_CHECK_THROWS(vista::devices::unitree_protocol::validate_frame(bytes));
}
