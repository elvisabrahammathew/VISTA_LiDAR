#include <atomic>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "1_Platform/pubsub/pubsub.hpp"
#include "4_Applications/logger/logger.hpp"
#include "unittest/test.hpp"

namespace {

std::filesystem::path temporary_output(const std::string& extension) {
    static std::atomic<std::uint64_t> next_id{0};
    return std::filesystem::temp_directory_path() /
           ("vista-edge-cpp-test-" + std::to_string(next_id++) + "." + extension);
}

}  // namespace

VISTA_TEST(logger_preserves_raw_bytes) {
    const auto path = temporary_output("bin");
    vista::platform::Topic<vista::devices::LidarRawMessage> topic("test/raw", 2);
    auto subscriber = topic.subscribe("raw-logger");
    {
        auto publisher = topic.publisher();
        publisher.publish(vista::devices::LidarRawMessage(
            "test",
            0,
            10,
            20,
            vista::devices::RawPacket({1, 2, 3, 4})));
    }

    const auto report =
        vista::application::run_raw_logger(std::move(subscriber), path);
    std::ifstream input(path, std::ios::binary);
    const std::vector<std::uint8_t> bytes{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    VISTA_CHECK(report.message_count == 1);
    VISTA_CHECK(bytes == std::vector<std::uint8_t>({1, 2, 3, 4}));
    input.close();
    std::filesystem::remove(path);
}

VISTA_TEST(logger_writes_pcd_header_and_point) {
    const auto path = temporary_output("pcd");
    vista::platform::Topic<vista::devices::LidarPointCloudMessage> topic(
        "test/pointcloud", 2);
    auto subscriber = topic.subscribe("pcd-logger");
    {
        auto publisher = topic.publisher();
        publisher.publish(vista::devices::LidarPointCloudMessage(
            "test",
            0,
            10,
            20,
            vista::models::PointCloudFrame(
                10,
                {vista::models::PointXYZIRT{1.0F, 2.0F, 3.0F, 4, 5, 0, 10}})));
    }

    const auto report =
        vista::application::run_pcd_logger(std::move(subscriber), path);
    std::ifstream input(path);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    VISTA_CHECK(report.message_count == 1);
    VISTA_CHECK(report.point_count == 1);
    VISTA_CHECK(contents.find("POINTS 1") != std::string::npos);
    VISTA_CHECK(
        contents.find("1.000000 2.000000 3.000000 4 5 0") != std::string::npos);
    input.close();
    std::filesystem::remove(path);
}
