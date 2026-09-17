#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "1_Platform/message_bus/message_bus.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"

namespace {

class MockReader final : public vista::devices::ILidarReader {
public:
    explicit MockReader(vista::platform::StopToken stop) : stop_(std::move(stop)) {}

    vista::devices::RawPacket read_raw_packet() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        stop_.request_stop();
        return vista::devices::RawPacket({1, 2, 3}, 100);
    }

private:
    vista::platform::StopToken stop_;
};

class MockDecoder final : public vista::devices::ILidarDecoder {
public:
    vista::models::PointCloudFrame decode_packet(
        const vista::devices::RawPacket& packet) override {
        VISTA_CHECK(packet.bytes() == std::vector<std::uint8_t>({1, 2, 3}));
        return {
            packet.timestamp_ns().value_or(0),
            {vista::models::PointXYZIRT{1.0F, 0.0F, 0.5F, 1, 0, 0, 100}},
        };
    }
};

}  // namespace

VISTA_TEST(workers_register_their_own_topics_and_process_one_packet) {
    vista::platform::StopToken stop;
    vista::platform::MessageBus bus;
    bus.configure_topic(vista::models::topics::lidar_raw, 2);
    bus.configure_topic(vista::models::topics::pointcloud_decoded, 2);
    bus.configure_topic(vista::models::topics::pointcloud_processed, 2);

    std::optional<vista::devices::LidarWorkerReport> read_report;
    std::optional<vista::devices::LidarWorkerReport> decode_report;
    std::optional<vista::application::PreprocessingReport> preprocessing_report;
    std::string read_error;
    std::string decode_error;
    std::string preprocessing_error;
    auto decoder_slot =
        std::make_shared<vista::devices::LidarDecoderSlot>();

    // Consumers are started first, but each worker performs its own subscribe
    // and publisher setup rather than receiving endpoints from this test.
    auto preprocessing = vista::application::spawn_preprocessing_worker(
        bus,
        vista::platform::ThreadConfig("test-preprocessing", 3),
        stop,
        vista::application::PreprocessingConfig{
            0.0F, 10.0F, std::nullopt, std::nullopt, std::nullopt},
        [&](auto report, auto error) {
            preprocessing_report = std::move(report);
            preprocessing_error = std::move(error);
        });
    auto decoder = vista::devices::spawn_lidar_decode_worker(
        bus,
        vista::platform::ThreadConfig("test-lidar-decode", 2),
        stop,
        decoder_slot,
        [&](auto report, auto error) {
            decode_report = std::move(report);
            decode_error = std::move(error);
        });
    auto reader = vista::devices::spawn_lidar_read_worker(
        bus,
        vista::platform::ThreadConfig("test-lidar-read", 1),
        stop,
        [stop]() {
            return vista::devices::Lidar::from_parts(
                "Mock LiDAR",
                "mock-lidar",
                std::make_unique<MockReader>(stop),
                std::make_unique<MockDecoder>());
        },
        std::chrono::milliseconds(1),
        decoder_slot,
        [&](auto report, auto error) {
            read_report = std::move(report);
            read_error = std::move(error);
        });

    reader.join();
    decoder.join();
    preprocessing.join();

    VISTA_CHECK(read_error.empty());
    VISTA_CHECK(decode_error.empty());
    VISTA_CHECK(preprocessing_error.empty());
    VISTA_CHECK(read_report && read_report->message_count == 1);
    VISTA_CHECK(decode_report && decode_report->message_count == 1);
    VISTA_CHECK(
        preprocessing_report && preprocessing_report->message_count == 1);
    VISTA_CHECK(preprocessing_report->point_count == 1);
}

VISTA_TEST(lidar_read_worker_retries_after_connection_failure) {
    vista::platform::StopToken stop;
    vista::platform::MessageBus bus;
    bus.configure_topic(vista::models::topics::lidar_raw, 2);

    std::atomic<int> connection_attempts{0};
    std::optional<vista::devices::LidarWorkerReport> read_report;
    std::string read_error;
    auto decoder_slot =
        std::make_shared<vista::devices::LidarDecoderSlot>();

    auto reader = vista::devices::spawn_lidar_read_worker(
        bus,
        vista::platform::ThreadConfig("test-lidar-reconnect", 1),
        stop,
        [&]() {
            if (++connection_attempts == 1) {
                throw std::runtime_error("mock LiDAR is disconnected");
            }
            return vista::devices::Lidar::from_parts(
                "Mock LiDAR",
                "mock-lidar",
                std::make_unique<MockReader>(stop),
                std::make_unique<MockDecoder>());
        },
        std::chrono::milliseconds(1),
        decoder_slot,
        [&](auto report, auto error) {
            read_report = std::move(report);
            read_error = std::move(error);
        });

    reader.join();

    VISTA_CHECK(connection_attempts == 2);
    VISTA_CHECK(read_error.empty());
    VISTA_CHECK(read_report && read_report->message_count == 1);
}
