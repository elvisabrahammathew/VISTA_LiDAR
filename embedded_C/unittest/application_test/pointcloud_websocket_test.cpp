#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "unittest/test.hpp"

namespace {

std::uint32_t read_u32_le(
    const std::vector<std::uint8_t>& value,
    std::size_t offset) {
    return static_cast<std::uint32_t>(value[offset]) |
           (static_cast<std::uint32_t>(value[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(value[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(value[offset + 3]) << 24U);
}

std::uint64_t read_u64_le(
    const std::vector<std::uint8_t>& value,
    std::size_t offset) {
    std::uint64_t result{};
    for (unsigned int shift = 0; shift < 64U; shift += 8U) {
        result |= static_cast<std::uint64_t>(value[offset + shift / 8U])
                  << shift;
    }
    return result;
}

float read_float_le(
    const std::vector<std::uint8_t>& value,
    std::size_t offset) {
    const auto bits = read_u32_le(value, offset);
    float result{};
    std::memcpy(&result, &bits, sizeof(result));
    return result;
}

vista::models::PointXYZIRT point(
    float x,
    float y,
    float z,
    std::uint8_t intensity) {
    return {x, y, z, intensity, 0, 0, 0};
}

}  // namespace

VISTA_TEST(pointcloud_websocket_encodes_lpc1_xyzi_and_samples_large_frames) {
    vista::devices::LidarPointCloudMessage message(
        "unitree-l2",
        42,
        998,
        999,
        vista::models::PointCloudFrame(
            123'456,
            {
                point(1.0F, 2.0F, 3.0F, 10),
                point(4.0F, 5.0F, 6.0F, 20),
                point(7.0F, 8.0F, 9.0F, 30),
            }));

    const auto packet =
        vista::application::encode_lpc1_pointcloud(message, 2);
    VISTA_CHECK(packet.size() == 32 + 2 * 16);
    VISTA_CHECK(std::string(packet.begin(), packet.begin() + 4) == "LPC1");
    VISTA_CHECK(packet[4] == 1);
    VISTA_CHECK(packet[5] == 1);
    VISTA_CHECK(packet[6] == 32 && packet[7] == 0);
    VISTA_CHECK(read_u64_le(packet, 8) == 42);
    VISTA_CHECK(read_u64_le(packet, 16) == 123'456);
    VISTA_CHECK(read_u32_le(packet, 24) == 2);
    VISTA_CHECK(read_u32_le(packet, 28) == 16);
    VISTA_CHECK_NEAR(read_float_le(packet, 32), 1.0F, 0.0001F);
    VISTA_CHECK_NEAR(read_float_le(packet, 44), 10.0F, 0.0001F);
    VISTA_CHECK_NEAR(read_float_le(packet, 48), 7.0F, 0.0001F);
    VISTA_CHECK_NEAR(read_float_le(packet, 60), 30.0F, 0.0001F);
}

VISTA_TEST(pointcloud_websocket_rejects_zero_maximum_points) {
    vista::devices::LidarPointCloudMessage message(
        "test",
        1,
        std::nullopt,
        2,
        vista::models::PointCloudFrame{});
    VISTA_CHECK_THROWS(
        vista::application::encode_lpc1_pointcloud(message, 0));
}
