#include <array>
#include <cstdint>
#include <vector>

#include "3_Devices/lidars/quanergym8/quanergym8.hpp"
#include "unittest/test.hpp"

namespace {

void write_u16_be(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value >> 8U);
    bytes[offset + 1] = static_cast<std::uint8_t>(value & 0xffU);
}

void write_u32_be(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint32_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value >> 24U);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 16U);
    bytes[offset + 2] = static_cast<std::uint8_t>(value >> 8U);
    bytes[offset + 3] = static_cast<std::uint8_t>(value);
}

vista::devices::RawPacket reduced_packet(
    std::uint16_t position,
    std::size_t laser,
    std::uint32_t distance) {
    std::vector<std::uint8_t> bytes(
        vista::devices::quanergy_m8_reduced_return_packet_size);
    write_u32_be(bytes, 0, vista::devices::quanergy_m8_packet_signature);
    write_u32_be(
        bytes,
        4,
        static_cast<std::uint32_t>(
            vista::devices::quanergy_m8_reduced_return_packet_size));
    write_u32_be(bytes, 8, 1);
    write_u32_be(bytes, 12, 2);
    bytes[19] = vista::devices::quanergy_m8_reduced_return_type;
    bytes[22] = 2;

    const auto firing_base = vista::devices::quanergy_m8_header_size + 4;
    write_u16_be(bytes, firing_base, position);
    write_u32_be(bytes, firing_base + 4 + laser * 4, distance);
    bytes[firing_base + 36 + laser] = 42;
    return vista::devices::RawPacket(std::move(bytes));
}

}  // namespace

VISTA_TEST(quanergy_validates_reduced_header) {
    std::array<std::uint8_t, vista::devices::quanergy_m8_header_size> header{};
    header[0] = 0x75;
    header[1] = 0xbd;
    header[2] = 0x7e;
    header[3] = 0x97;
    const auto size = vista::devices::quanergy_m8_reduced_return_packet_size;
    header[4] = static_cast<std::uint8_t>(size >> 24U);
    header[5] = static_cast<std::uint8_t>(size >> 16U);
    header[6] = static_cast<std::uint8_t>(size >> 8U);
    header[7] = static_cast<std::uint8_t>(size);
    header[19] = vista::devices::quanergy_m8_reduced_return_type;
    VISTA_CHECK(vista::devices::QuanergyM8::validate_header(header) == size);
}

VISTA_TEST(quanergy_rejects_unknown_packet_type) {
    std::array<std::uint8_t, vista::devices::quanergy_m8_header_size> header{};
    header[0] = 0x75;
    header[1] = 0xbd;
    header[2] = 0x7e;
    header[3] = 0x97;
    header[19] = 0xff;
    VISTA_CHECK_THROWS(vista::devices::QuanergyM8::validate_header(header));
}

VISTA_TEST(quanergy_converts_one_meter_on_positive_x) {
    vista::devices::QuanergyM8Decoder decoder;
    const auto frame = decoder.decode_packet(reduced_packet(0, 6, 100'000));
    VISTA_CHECK(frame.points.size() == 1);
    VISTA_CHECK_NEAR(frame.points[0].x, 1.0F, 1.0e-6F);
    VISTA_CHECK_NEAR(frame.points[0].y, 0.0F, 1.0e-6F);
    VISTA_CHECK_NEAR(frame.points[0].z, 0.0F, 1.0e-6F);
    VISTA_CHECK(frame.points[0].intensity == 42);
    VISTA_CHECK(frame.points[0].ring == 6);
    VISTA_CHECK(frame.points[0].return_id == 2);
}

VISTA_TEST(quanergy_position_2600_points_on_positive_y) {
    vista::devices::QuanergyM8Decoder decoder;
    const auto frame = decoder.decode_packet(reduced_packet(2600, 6, 100'000));
    VISTA_CHECK(frame.points.size() == 1);
    VISTA_CHECK_NEAR(frame.points[0].x, 0.0F, 1.0e-6F);
    VISTA_CHECK_NEAR(frame.points[0].y, 1.0F, 1.0e-6F);
}
