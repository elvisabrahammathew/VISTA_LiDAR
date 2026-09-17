#include "4_Applications/logger/logger.hpp"

#include <exception>
#include <stdexcept>

#include "2_Transport/storage/local.hpp"
#include "models/topics.hpp"

namespace vista::application {

namespace {

std::string current_exception_message() {
    try {
        throw;
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown logger failure";
    }
}

}  // namespace

LoggerReport run_raw_logger(
    platform::TopicSubscriber<devices::LidarRawMessage> subscriber,
    const std::filesystem::path& path) {
    transport::RawCaptureWriter writer(path);
    LoggerReport report;
    std::shared_ptr<const devices::LidarRawMessage> message;
    while (subscriber.receive(message) == platform::ReceiveStatus::message) {
        writer.write_bytes(message->payload.bytes());
        ++report.message_count;
    }
    report.dropped_message_count = subscriber.dropped_messages();
    writer.finish();
    return report;
}

LoggerReport run_pcd_logger(
    platform::TopicSubscriber<devices::LidarPointCloudMessage> subscriber,
    const std::filesystem::path& path) {
    transport::PcdWriter writer(path);
    LoggerReport report;
    std::shared_ptr<const devices::LidarPointCloudMessage> message;
    while (subscriber.receive(message) == platform::ReceiveStatus::message) {
        writer.write_frame(message->payload);
        ++report.message_count;
        report.point_count += message->payload.points.size();
    }
    report.dropped_message_count = subscriber.dropped_messages();
    const auto written_points = writer.finish();
    if (written_points != report.point_count) {
        throw std::runtime_error("PCD writer point count mismatch");
    }
    return report;
}

platform::WorkerHandle spawn_raw_logger(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::filesystem::path path,
    LoggerCompletion on_complete) {
    // This worker declares and owns its subscription to the RAW topic.
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto raw_input = inputs.subscribe<devices::LidarRawMessage>(
        bus, models::topics::lidar_raw);
    auto subscriber = std::move(raw_input).take_subscriber();
    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         subscriber = std::move(subscriber),
         path = std::move(path),
         on_complete = std::move(on_complete)]() mutable {
            try {
                on_complete(run_raw_logger(std::move(subscriber), path), {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

platform::WorkerHandle spawn_pcd_logger(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::filesystem::path path,
    LoggerCompletion on_complete) {
    // PCD output is intentionally based on the fully processed point cloud.
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto processed_input = inputs.subscribe<devices::LidarPointCloudMessage>(
        bus, models::topics::pointcloud_processed);
    auto subscriber = std::move(processed_input).take_subscriber();
    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         subscriber = std::move(subscriber),
         path = std::move(path),
         on_complete = std::move(on_complete)]() mutable {
            try {
                on_complete(run_pcd_logger(std::move(subscriber), path), {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
