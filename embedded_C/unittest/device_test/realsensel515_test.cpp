#include <array>
#include <vector>

#include "3_Devices/lidars/realsensel515/realsensel515.hpp"
#include "unittest/test.hpp"

VISTA_TEST(realsense_ignores_zero_depth_pixels) {
    const std::vector<std::uint8_t> bytes{0, 0};
    const std::vector<std::array<float, 3>> rays{{{0.0F, 0.0F, 1.0F}}};
    const auto frame =
        vista::devices::decode_l515_depth_frame(bytes, rays, 0.001F, 10);
    VISTA_CHECK(frame.points.empty());
    VISTA_CHECK(frame.timestamp_ns == 10);
}

VISTA_TEST(realsense_converts_optical_coordinates) {
    const auto depth = static_cast<std::uint16_t>(1000);
    const std::vector<std::uint8_t> bytes{
        static_cast<std::uint8_t>(depth & 0xffU),
        static_cast<std::uint8_t>(depth >> 8U),
    };
    const std::vector<std::array<float, 3>> rays{{{0.25F, 0.5F, 1.0F}}};
    const auto frame =
        vista::devices::decode_l515_depth_frame(bytes, rays, 0.001F, 20);

    VISTA_CHECK(frame.points.size() == 1);
    VISTA_CHECK_NEAR(frame.points[0].x, 1.0F, 1.0e-6F);
    VISTA_CHECK_NEAR(frame.points[0].y, -0.25F, 1.0e-6F);
    VISTA_CHECK_NEAR(frame.points[0].z, -0.5F, 1.0e-6F);
    VISTA_CHECK(frame.points[0].timestamp_ns == 20);
}

VISTA_TEST(realsense_rejects_incorrect_depth_buffer_size) {
    const std::vector<std::array<float, 3>> rays{{{0.0F, 0.0F, 1.0F}}};
    VISTA_CHECK_THROWS(vista::devices::decode_l515_depth_frame(
        std::vector<std::uint8_t>{1}, rays, 0.001F, 0));
}
