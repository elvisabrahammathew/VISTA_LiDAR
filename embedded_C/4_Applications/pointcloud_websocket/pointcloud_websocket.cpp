#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"

#include <algorithm>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include "2_Transport/messaging/websocket_server.hpp"
#include "models/topics.hpp"

namespace vista::application {

namespace {

constexpr std::size_t lpc1_base_header_size = 32;
constexpr std::size_t lpc1_ground_header_size = 96;
constexpr std::size_t lpc1_point_stride = 16;
constexpr std::size_t absolute_maximum_points = 2'000'000;

void append_u16_le(std::vector<std::uint8_t>& output, std::uint16_t value) {
    output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32_le(std::vector<std::uint8_t>& output, std::uint32_t value) {
    for (unsigned int shift = 0; shift < 32U; shift += 8U) {
        output.push_back(
            static_cast<std::uint8_t>((value >> shift) & 0xFFU));
    }
}

void append_u64_le(std::vector<std::uint8_t>& output, std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64U; shift += 8U) {
        output.push_back(
            static_cast<std::uint8_t>((value >> shift) & 0xFFU));
    }
}

void append_float_le(std::vector<std::uint8_t>& output, float value) {
    std::uint32_t bits{};
    static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit");
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32_le(output, bits);
}

std::string current_exception_message() {
    try {
        throw;
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown point-cloud WebSocket failure";
    }
}

}  // namespace

void validate_pointcloud_websocket_config(
    const PointCloudWebSocketConfig& config) {
    if (config.bind_address.empty()) {
        throw std::invalid_argument(
            "point-cloud WebSocket bind address cannot be empty");
    }
    if (config.port == 0) {
        throw std::invalid_argument(
            "point-cloud WebSocket port must be between 1 and 65535");
    }
    if (config.maximum_points == 0 ||
        config.maximum_points > absolute_maximum_points) {
        throw std::invalid_argument(
            "point-cloud WebSocket maximum points must be between 1 and " +
            std::to_string(absolute_maximum_points));
    }
    if (config.maximum_clients == 0 || config.maximum_clients > 64) {
        throw std::invalid_argument(
            "point-cloud WebSocket maximum clients must be between 1 and 64");
    }
    if (config.publish_interval.count() <= 0) {
        throw std::invalid_argument(
            "point-cloud WebSocket publish interval must be positive");
    }
    if (config.retry_interval.count() <= 0) {
        throw std::invalid_argument(
            "point-cloud WebSocket retry interval must be positive");
    }
}

std::vector<std::uint8_t> encode_lpc1_pointcloud(
    const devices::LidarPointCloudMessage& message,
    std::size_t maximum_points,
    const models::GroundStatus* ground_status) {
    if (maximum_points == 0) {
        throw std::invalid_argument("LPC1 maximum points cannot be zero");
    }

    const auto& points = message.payload.points;
    const auto sample_step = points.size() > maximum_points
                                 ? (points.size() + maximum_points - 1U) /
                                       maximum_points
                                 : 1U;
    const auto encoded_count =
        points.empty() ? 0U : (points.size() + sample_step - 1U) / sample_step;
    if (encoded_count > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("LPC1 point count exceeds uint32_t");
    }

    std::vector<std::uint8_t> output;
    const auto header_size=ground_status ? lpc1_ground_header_size : lpc1_base_header_size;
    output.reserve(header_size + encoded_count * lpc1_point_stride);
    output.insert(output.end(), {'L', 'P', 'C', '1'});
    output.push_back(1U);  // Protocol version.
    output.push_back(static_cast<std::uint8_t>(1U | (ground_status ? 2U : 0U)));
    append_u16_le(output, static_cast<std::uint16_t>(header_size));
    append_u64_le(output, message.sequence);
    const auto timestamp = message.payload.timestamp_ns != 0
                               ? message.payload.timestamp_ns
                               : message.sensor_timestamp_ns.value_or(
                                     message.received_timestamp_ns);
    append_u64_le(output, timestamp);
    append_u32_le(output, static_cast<std::uint32_t>(encoded_count));
    append_u32_le(output, static_cast<std::uint32_t>(lpc1_point_stride));

    if (ground_status) {
        output.push_back(static_cast<std::uint8_t>(ground_status->state));
        output.push_back(static_cast<std::uint8_t>(
            (ground_status->calibrated ? 1U : 0U) |
            (ground_status->using_imu ? 2U : 0U) |
            (ground_status->using_static_fallback ? 4U : 0U)));
        const auto mode=ground_status->configured_mode=="static" ? 0U :
            ground_status->configured_mode=="ransac" ? 1U :
            ground_status->configured_mode=="hybrid" ? 2U : 255U;
        output.push_back(static_cast<std::uint8_t>(mode));
        output.push_back(0U);
        append_float_le(output,ground_status->plane_a);
        append_float_le(output,ground_status->plane_b);
        append_float_le(output,ground_status->plane_c);
        append_float_le(output,ground_status->plane_d);
        append_float_le(output,ground_status->ground_tilt_deg);
        append_float_le(output,ground_status->sensor_to_ground_distance_m);
        append_float_le(output,ground_status->expected_ground_distance_m);
        append_float_le(output,ground_status->ground_height_error_m);
        append_float_le(output,ground_status->ground_inlier_ratio);
        append_float_le(output,ground_status->mean_residual_m);
        append_float_le(output,ground_status->rms_residual_m);
        append_float_le(output,ground_status->p95_residual_m);
        append_float_le(output,ground_status->removed_ground_ratio);
        append_u32_le(output,static_cast<std::uint32_t>(std::min<std::uint64_t>(ground_status->input_point_count,std::numeric_limits<std::uint32_t>::max())));
        append_u32_le(output,static_cast<std::uint32_t>(std::min<std::uint64_t>(ground_status->removed_ground_point_count,std::numeric_limits<std::uint32_t>::max())));
    }

    for (std::size_t index = 0; index < points.size(); index += sample_step) {
        const auto& point = points[index];
        append_float_le(output, point.x);
        append_float_le(output, point.y);
        append_float_le(output, point.z);
        append_float_le(output, static_cast<float>(point.intensity));
    }
    return output;
}

platform::WorkerHandle spawn_pointcloud_websocket(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    PointCloudWebSocketConfig config,
    PointCloudWebSocketCompletion on_complete) {
    validate_pointcloud_websocket_config(config);
    auto subscriber = bus.subscribe<devices::LidarPointCloudMessage>(
        models::topics::pointcloud_processed,
        thread_config.name);
    auto ground_subscriber = bus.subscribe<models::LidarGroundStatusMessage>(
        models::topics::ground_status,
        thread_config.name);

    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         subscriber = std::move(subscriber),
         ground_subscriber = std::move(ground_subscriber),
         config = std::move(config),
         on_complete = std::move(on_complete)]() mutable {
            try {
                PointCloudWebSocketReport report;
                std::unique_ptr<transport::WebSocketServer> server;
                auto next_listen_attempt = std::chrono::steady_clock::now();
                auto next_publish =
                    std::chrono::steady_clock::now() + config.publish_interval;
                std::vector<models::PointXYZIRT> accumulated_points;
                accumulated_points.reserve(config.maximum_points);
                std::shared_ptr<const devices::LidarPointCloudMessage>
                    latest_full_frame;
                std::string latest_lidar_id;
                std::uint64_t latest_sequence{};
                std::optional<std::uint64_t> latest_sensor_timestamp;
                std::uint64_t latest_received_timestamp{};
                std::uint64_t latest_frame_timestamp{};
                const auto full_frame_threshold = std::max<std::size_t>(
                    1'000,
                    config.maximum_points / 4U);
                const auto subscriber_wait = std::min(
                    config.publish_interval,
                    std::chrono::milliseconds(100));
                bool topic_closed = false;
                std::shared_ptr<const models::LidarGroundStatusMessage> latest_ground;

                const auto broadcast =
                    [&](const devices::LidarPointCloudMessage& cloud) {
                        if (!server || server->client_count() == 0) {
                            return;
                        }
                        const auto packet = encode_lpc1_pointcloud(
                            cloud,
                            config.maximum_points,
                            latest_ground ? &latest_ground->payload : nullptr);
                        const auto deliveries = server->broadcast_binary(
                            packet.data(),
                            packet.size());
                        report.client_deliveries += deliveries;
                        if (deliveries > 0) {
                            ++report.broadcast_frames;
                            const auto step=cloud.payload.points.size()>config.maximum_points
                                ? (cloud.payload.points.size()+config.maximum_points-1U)/config.maximum_points : 1U;
                            report.encoded_points += cloud.payload.points.empty() ? 0U :
                                (cloud.payload.points.size()+step-1U)/step;
                        }
                    };

                while (!stop.is_stop_requested() && !topic_closed) {
                    std::shared_ptr<const models::LidarGroundStatusMessage> ground_message;
                    while (ground_subscriber.try_receive(ground_message)==platform::ReceiveStatus::message) {
                        latest_ground=ground_message;
                    }
                    const auto now = std::chrono::steady_clock::now();
                    if (!server && now >= next_listen_attempt) {
                        try {
                            auto listening = transport::WebSocketServer::listen({
                                config.bind_address,
                                config.port,
                                config.maximum_clients,
                                std::chrono::milliseconds(1'000),
                                std::chrono::milliseconds(250),
                            });
                            server = std::make_unique<transport::WebSocketServer>(
                                std::move(listening));
                            std::cout
                                << "Point-cloud WebSocket listening at ws://"
                                << config.bind_address << ':' << config.port
                                << '\n';
                        } catch (const std::exception& error) {
                            ++report.server_failures;
                            std::cerr
                                << "Point-cloud WebSocket listen failed: "
                                << error.what() << "; retrying in "
                                << config.retry_interval.count() << " ms\n";
                            next_listen_attempt = now + config.retry_interval;
                        }
                    }

                    if (server) {
                        try {
                            const auto accepted = server->poll_accept();
                            if (accepted > 0) {
                                std::cout
                                    << "Point-cloud WebSocket client connected; "
                                    << server->client_count()
                                    << " active client(s)\n";
                            }
                        } catch (const std::exception& error) {
                            ++report.server_failures;
                            std::cerr
                                << "Point-cloud WebSocket server error: "
                                << error.what() << "; restarting server\n";
                            server.reset();
                            next_listen_attempt = now + config.retry_interval;
                        }
                    }

                    std::shared_ptr<const devices::LidarPointCloudMessage> message;
                    const auto status = subscriber.receive_for(
                        message,
                        subscriber_wait);
                    if (status == platform::ReceiveStatus::closed) {
                        topic_closed = true;
                    } else if (
                        status == platform::ReceiveStatus::message && message) {
                        ++report.received_messages;
                        if (message->payload.points.size() >=
                            full_frame_threshold) {
                            // Depth cameras already deliver complete frames;
                            // keep only the latest one during this interval.
                            accumulated_points.clear();
                            latest_full_frame = message;
                        } else {
                            // Packetized spinning LiDARs need a short window
                            // of processed packets to form a visible scan.
                            latest_full_frame.reset();
                            const auto remaining =
                                config.maximum_points -
                                accumulated_points.size();
                            const auto count = std::min(
                                remaining,
                                message->payload.points.size());
                            accumulated_points.insert(
                                accumulated_points.end(),
                                message->payload.points.begin(),
                                message->payload.points.begin() +
                                    static_cast<std::ptrdiff_t>(count));
                            latest_lidar_id = message->lidar_id;
                            latest_sequence = message->sequence;
                            latest_sensor_timestamp =
                                message->sensor_timestamp_ns;
                            latest_received_timestamp =
                                message->received_timestamp_ns;
                            latest_frame_timestamp =
                                message->payload.timestamp_ns;
                        }
                    }

                    // Ground status is published immediately before the
                    // processed cloud, so drain it again to attach the model
                    // belonging to the frame that just woke this worker.
                    while (ground_subscriber.try_receive(ground_message)==platform::ReceiveStatus::message) {
                        latest_ground=ground_message;
                    }

                    const auto publish_now = std::chrono::steady_clock::now();
                    if (publish_now >= next_publish) {
                        if (latest_full_frame) {
                            broadcast(*latest_full_frame);
                        } else if (!accumulated_points.empty()) {
                            devices::LidarPointCloudMessage aggregate(
                                latest_lidar_id,
                                latest_sequence,
                                latest_sensor_timestamp,
                                latest_received_timestamp,
                                models::PointCloudFrame(
                                    latest_frame_timestamp,
                                    std::move(accumulated_points)));
                            broadcast(aggregate);
                            accumulated_points.clear();
                            accumulated_points.reserve(config.maximum_points);
                        }
                        latest_full_frame.reset();
                        accumulated_points.clear();
                        next_publish = publish_now + config.publish_interval;
                    }
                }

                report.dropped_input_messages = subscriber.dropped_messages()+ground_subscriber.dropped_messages();
                on_complete(report, {});
            } catch (...) {
                // Encoding/programming failures are reported to main, but an
                // optional visualization output must never stop LiDAR capture.
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
