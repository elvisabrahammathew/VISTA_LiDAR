#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "models/lidars/lidar_message.hpp"
#include "models/lidars/pointcloud.hpp"

namespace vista::devices {

enum class LidarType {
    quanergy_m8,
    realsense_l515,
    unitree_4d,
};

std::string to_string(LidarType type);
LidarType parse_lidar_type(const std::string& value);

struct DepthStreamConfig {
    /// Optional librealsense USB device serial; empty selects the first L515.
    std::optional<std::string> usb_serial;
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t frames_per_second{};
    std::chrono::milliseconds frame_timeout{};
};

struct LidarConfig {
    LidarType lidar_type{LidarType::quanergy_m8};
    std::string sensor_ip{"192.168.1.3"};
    std::optional<std::uint16_t> port;
    std::optional<DepthStreamConfig> depth_stream;
    /// Maximum time allowed for connection and for receiving the next sample.
    std::chrono::milliseconds connection_timeout{5'000};
};

/// Owns one complete packet exactly as it arrived from a sensor.
class RawPacket {
public:
    explicit RawPacket(std::vector<std::uint8_t> bytes)
        : bytes_(std::move(bytes)) {}

    RawPacket(std::vector<std::uint8_t> bytes, std::uint64_t timestamp_ns)
        : bytes_(std::move(bytes)), timestamp_ns_(timestamp_ns) {}

    const std::vector<std::uint8_t>& bytes() const noexcept { return bytes_; }
    std::size_t size() const noexcept { return bytes_.size(); }
    bool empty() const noexcept { return bytes_.empty(); }
    std::optional<std::uint64_t> timestamp_ns() const noexcept {
        return timestamp_ns_;
    }

private:
    std::vector<std::uint8_t> bytes_;
    std::optional<std::uint64_t> timestamp_ns_;
};

using LidarRawMessage = models::LidarMessage<RawPacket>;
using LidarPointCloudMessage = models::LidarMessage<models::PointCloudFrame>;

struct LidarWorkerReport {
    std::uint64_t message_count{};
    std::uint64_t dropped_message_count{};
};

class ILidarReader {
public:
    virtual ~ILidarReader() = default;
    virtual RawPacket read_raw_packet() = 0;
};

class ILidarDecoder {
public:
    virtual ~ILidarDecoder() = default;
    virtual models::PointCloudFrame decode_packet(const RawPacket& packet) = 0;
};

/// Shares the decoder created by the current LiDAR connection with the
/// independent decode worker. Reconnection can safely replace the decoder.
class LidarDecoderSlot {
public:
    void replace(std::unique_ptr<ILidarDecoder> decoder);
    models::PointCloudFrame decode_packet(const RawPacket& packet) const;

private:
    mutable std::mutex mutex_;
    std::shared_ptr<ILidarDecoder> decoder_;
};

struct LidarParts {
    std::string device_name;
    std::string lidar_id;
    std::unique_ptr<ILidarReader> reader;
    std::unique_ptr<ILidarDecoder> decoder;
};

class Lidar {
public:
    static Lidar connect(const LidarConfig& config);

    static Lidar from_parts(
        std::string device_name,
        std::string lidar_id,
        std::unique_ptr<ILidarReader> reader,
        std::unique_ptr<ILidarDecoder> decoder);

    const std::string& device_name() const noexcept { return device_name_; }
    LidarParts into_parts() &&;

private:
    Lidar(
        std::string device_name,
        std::string lidar_id,
        std::unique_ptr<ILidarReader> reader,
        std::unique_ptr<ILidarDecoder> decoder);

    std::string device_name_;
    std::string lidar_id_;
    std::unique_ptr<ILidarReader> reader_;
    std::unique_ptr<ILidarDecoder> decoder_;
};

/// Creates a fresh reader and decoder whenever the read worker reconnects.
using LidarConnector = std::function<Lidar()>;

using LidarCompletion =
    std::function<void(std::optional<LidarWorkerReport>, std::string)>;

platform::WorkerHandle spawn_lidar_read_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    LidarConnector connect_lidar,
    std::chrono::milliseconds reconnect_interval,
    std::shared_ptr<LidarDecoderSlot> decoder_slot,
    LidarCompletion on_complete);

platform::WorkerHandle spawn_lidar_decode_worker(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    std::shared_ptr<LidarDecoderSlot> decoder_slot,
    LidarCompletion on_complete);

}  // namespace vista::devices
