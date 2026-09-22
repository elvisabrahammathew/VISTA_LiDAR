#include "3_Devices/lidars/lidar.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "3_Devices/lidars/quanergym8/quanergym8.hpp"
#include "3_Devices/lidars/realsensel515/realsensel515.hpp"
#include "3_Devices/lidars/unitree4d/unitree4d.hpp"
#include "models/topics.hpp"

namespace vista::devices {

namespace {

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

std::string current_exception_message() {
    try {
        throw;
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown worker failure";
    }
}

std::uint64_t system_timestamp_ns() {
    const auto elapsed = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
}

void wait_before_reconnect(
    std::chrono::milliseconds interval,
    const platform::StopToken& stop) {
    // Wake regularly so Ctrl+C does not have to wait for a long retry delay.
    const auto deadline = std::chrono::steady_clock::now() + interval;
    while (!stop.is_stop_requested()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return;
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        std::this_thread::sleep_for(
            std::min(remaining, std::chrono::milliseconds(100)));
    }
}

LidarWorkerReport run_lidar_reader(
    LidarConnector connect_lidar,
    platform::TopicPublisher<LidarRawMessage> publisher,
    std::chrono::milliseconds reconnect_interval,
    const std::shared_ptr<LidarDecoderSlot>& decoder_slot,
    const platform::StopToken& stop) {
    LidarWorkerReport report;

    while (!stop.is_stop_requested()) {
        bool connected = false;
        try {
            auto lidar = connect_lidar();
            auto parts = std::move(lidar).into_parts();
            if (!parts.reader || !parts.decoder) {
                throw std::runtime_error(
                    "LiDAR connector returned an incomplete driver");
            }

            const auto device_name = parts.device_name;
            const auto lidar_id = parts.lidar_id;
            decoder_slot->replace(std::move(parts.decoder));
            connected = true;
            std::cout << "LiDAR connected: " << device_name << '\n';

            // No separate health poll is needed while data is arriving. A
            // read timeout or transport error leaves this loop and reconnects.
            while (!stop.is_stop_requested()) {
                auto packet = parts.reader->read_raw_packet();
                const auto sensor_timestamp = packet.timestamp_ns();
                publisher.publish(LidarRawMessage(
                    lidar_id,
                    report.message_count,
                    sensor_timestamp,
                    system_timestamp_ns(),
                    std::move(packet)));
                ++report.message_count;
            }
        } catch (const std::exception& error) {
            if (stop.is_stop_requested()) {
                break;
            }
            std::cerr << (connected ? "LiDAR connection lost: "
                                     : "LiDAR connection failed: ")
                      << error.what() << '\n';
        } catch (...) {
            if (stop.is_stop_requested()) {
                break;
            }
            std::cerr << (connected ? "LiDAR connection lost"
                                     : "LiDAR connection failed")
                      << ": unknown device error\n";
        }

        if (!stop.is_stop_requested()) {
            std::cerr << "Retrying LiDAR connection in "
                      << static_cast<double>(reconnect_interval.count()) / 1000.0
                      << " second(s).\n";
            wait_before_reconnect(reconnect_interval, stop);
        }
    }
    return report;
}

LidarWorkerReport run_lidar_decoder(
    const std::shared_ptr<LidarDecoderSlot>& decoder_slot,
    platform::TopicSubscriber<LidarRawMessage> subscriber,
    platform::TopicPublisher<LidarPointCloudMessage> pointcloud_publisher,
    platform::TopicPublisher<LidarImuMessage> imu_publisher) {
    LidarWorkerReport report;
    std::shared_ptr<const LidarRawMessage> message;
    while (subscriber.receive(message) == platform::ReceiveStatus::message) {
        if (auto imu = decoder_slot->decode_imu_packet(message->payload)) {
            imu_publisher.publish(LidarImuMessage(
                message->lidar_id,
                message->sequence,
                message->sensor_timestamp_ns,
                message->received_timestamp_ns,
                std::move(*imu)));
            ++report.message_count;
            continue;
        }
        auto frame = decoder_slot->decode_packet(message->payload);
        // Some sensors need several scan rows before one complete cloud exists.
        if (frame.points.empty()) {
            continue;
        }
        pointcloud_publisher.publish(LidarPointCloudMessage(
            message->lidar_id,
            message->sequence,
            message->sensor_timestamp_ns,
            message->received_timestamp_ns,
            std::move(frame)));
        ++report.message_count;
    }
    report.dropped_message_count = subscriber.dropped_messages();
    return report;
}

}  // namespace

std::string to_string(LidarType type) {
    switch (type) {
        case LidarType::quanergy_m8:
            return "quanergy-m8";
        case LidarType::realsense_l515:
            return "realsense-l515";
        case LidarType::unitree_l2:
            return "unitree-l2";
    }
    throw std::invalid_argument("unknown LiDAR type");
}

LidarType parse_lidar_type(const std::string& value) {
    const auto normalized = lower_copy(value);
    if (normalized == "quanergy-m8" || normalized == "quanergym8" ||
        normalized == "quanergy" || normalized == "m8") {
        return LidarType::quanergy_m8;
    }
    if (normalized == "realsense-l515" || normalized == "realsensel515" ||
        normalized == "realsense" || normalized == "l515") {
        return LidarType::realsense_l515;
    }
    if (normalized == "unitree-l2" || normalized == "unitreel2" ||
        normalized == "unitree-4d" || normalized == "unitree4d" ||
        normalized == "unitree" || normalized == "l2") {
        return LidarType::unitree_l2;
    }
    throw std::invalid_argument(
        "unsupported LiDAR type '" + value +
        "'; supported values: quanergy-m8, realsense-l515, unitree-l2");
}

std::string to_string(UnitreeConnectionMode mode) {
    switch (mode) {
        case UnitreeConnectionMode::automatic:
            return "auto";
        case UnitreeConnectionMode::serial:
            return "serial";
        case UnitreeConnectionMode::udp:
            return "udp";
    }
    throw std::invalid_argument("unknown Unitree connection mode");
}

UnitreeConnectionMode parse_unitree_connection_mode(const std::string& value) {
    const auto normalized = lower_copy(value);
    if (normalized == "auto" || normalized == "automatic") {
        return UnitreeConnectionMode::automatic;
    }
    if (normalized == "serial" || normalized == "usb") {
        return UnitreeConnectionMode::serial;
    }
    if (normalized == "udp" || normalized == "ethernet") {
        return UnitreeConnectionMode::udp;
    }
    throw std::invalid_argument(
        "unsupported Unitree connection mode '" + value +
        "'; supported values: auto, serial, udp");
}

Lidar::Lidar(
    std::string device_name,
    std::string lidar_id,
    std::unique_ptr<ILidarReader> reader,
    std::unique_ptr<ILidarDecoder> decoder)
    : device_name_(std::move(device_name)),
      lidar_id_(std::move(lidar_id)),
      reader_(std::move(reader)),
      decoder_(std::move(decoder)) {}

Lidar Lidar::connect(const LidarConfig& config) {
    switch (config.lidar_type) {
        case LidarType::quanergy_m8: {
            auto [reader, decoder] = QuanergyM8::connect(
                config.sensor_ip,
                config.port.value_or(quanergy_m8_default_port),
                config.connection_timeout);
            return Lidar(
                "Quanergy M8",
                "quanergy-m8",
                std::move(reader),
                std::move(decoder));
        }
        case LidarType::realsense_l515: {
            if (!config.depth_stream) {
                throw std::invalid_argument(
                    "RealSense L515 requires a depth-stream configuration");
            }
            auto [reader, decoder] = RealSenseL515::connect(*config.depth_stream);
            return Lidar(
                "Intel RealSense L515",
                "realsense-l515",
                std::move(reader),
                std::move(decoder));
        }
        case LidarType::unitree_l2: {
            if (!config.unitree_l2) {
                throw std::invalid_argument(
                    "Unitree L2 requires a serial/UDP configuration");
            }
            auto [reader, decoder] = UnitreeL2::connect(
                *config.unitree_l2, config.connection_timeout);
            return Lidar(
                "Unitree L2",
                "unitree-l2",
                std::move(reader),
                std::move(decoder));
        }
    }
    throw std::invalid_argument("unknown LiDAR type");
}

Lidar Lidar::from_parts(
    std::string device_name,
    std::string lidar_id,
    std::unique_ptr<ILidarReader> reader,
    std::unique_ptr<ILidarDecoder> decoder) {
    return Lidar(
        std::move(device_name),
        std::move(lidar_id),
        std::move(reader),
        std::move(decoder));
}

void LidarDecoderSlot::replace(std::unique_ptr<ILidarDecoder> decoder) {
    if (!decoder) {
        throw std::invalid_argument("LiDAR decoder cannot be null");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    decoder_ = std::shared_ptr<ILidarDecoder>(std::move(decoder));
}

std::optional<models::ImuFrame> LidarDecoderSlot::decode_imu_packet(
    const RawPacket& packet) const {
    std::shared_ptr<ILidarDecoder> decoder;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        decoder = decoder_;
    }
    if (!decoder) {
        throw std::runtime_error("LiDAR decoder is unavailable");
    }
    return decoder->decode_imu_packet(packet);
}

models::PointCloudFrame LidarDecoderSlot::decode_packet(
    const RawPacket& packet) const {
    std::shared_ptr<ILidarDecoder> decoder;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        decoder = decoder_;
    }
    if (!decoder) {
        throw std::runtime_error("LiDAR decoder is unavailable");
    }
    return decoder->decode_packet(packet);
}

LidarParts Lidar::into_parts() && {
    return {
        std::move(device_name_),
        std::move(lidar_id_),
        std::move(reader_),
        std::move(decoder_),
    };
}

platform::WorkerHandle spawn_lidar_read_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    LidarConnector connect_lidar,
    std::chrono::milliseconds reconnect_interval,
    std::shared_ptr<LidarDecoderSlot> decoder_slot,
    LidarCompletion on_complete) {
    if (!connect_lidar || !decoder_slot || reconnect_interval.count() <= 0) {
        throw std::invalid_argument("invalid reconnecting LiDAR worker configuration");
    }
    // The reader owns the publisher endpoint for the shared LiDAR RAW topic.
    auto publisher =
        bus.publisher<LidarRawMessage>(models::topics::lidar_raw);
    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         connect_lidar = std::move(connect_lidar),
         publisher = std::move(publisher),
         reconnect_interval,
         decoder_slot = std::move(decoder_slot),
         on_complete = std::move(on_complete)]() mutable {
            try {
                auto report = run_lidar_reader(
                    std::move(connect_lidar),
                    std::move(publisher),
                    reconnect_interval,
                    decoder_slot,
                    stop);
                on_complete(report, {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

platform::WorkerHandle spawn_lidar_decode_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::shared_ptr<LidarDecoderSlot> decoder_slot,
    LidarCompletion on_complete) {
    if (!decoder_slot) {
        throw std::invalid_argument("LiDAR decoder slot cannot be null");
    }
    // Topic dependencies stay with this worker instead of being wired in main.
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto raw_input =
        inputs.subscribe<LidarRawMessage>(bus, models::topics::lidar_raw);
    auto subscriber = std::move(raw_input).take_subscriber();
    auto pointcloud_publisher = bus.publisher<LidarPointCloudMessage>(
        models::topics::pointcloud_decoded);
    auto imu_publisher =
        bus.publisher<LidarImuMessage>(models::topics::lidar_imu);
    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         decoder_slot = std::move(decoder_slot),
         subscriber = std::move(subscriber),
         pointcloud_publisher = std::move(pointcloud_publisher),
         imu_publisher = std::move(imu_publisher),
         on_complete = std::move(on_complete)]() mutable {
            try {
                auto report = run_lidar_decoder(
                    decoder_slot,
                    std::move(subscriber),
                    std::move(pointcloud_publisher),
                    std::move(imu_publisher));
                on_complete(report, {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::devices
