#include "3_Devices/lidars/unitree4d/unitree4d.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "2_Transport/peripheral/serial/serial.hpp"
#include "2_Transport/peripheral/udp/udp.hpp"
#include "3_Devices/lidars/unitree4d/unitree_protocol.hpp"

namespace vista::devices {

namespace {

constexpr std::array<std::uint8_t, 4> frame_magic{0x55, 0xAA, 0x05, 0x0A};
constexpr std::size_t read_buffer_size = 65'536;

class IUnitreeTransport {
public:
    virtual ~IUnitreeTransport() = default;
    virtual std::size_t read(std::uint8_t* output, std::size_t capacity) = 0;
};

class SerialUnitreeTransport final : public IUnitreeTransport {
public:
    explicit SerialUnitreeTransport(transport::SerialConnection connection)
        : connection_(std::move(connection)) {}
    std::size_t read(std::uint8_t* output, std::size_t capacity) override {
        return connection_.read_some(output, capacity);
    }

private:
    transport::SerialConnection connection_;
};

class UdpUnitreeTransport final : public IUnitreeTransport {
public:
    explicit UdpUnitreeTransport(transport::UdpConnection connection)
        : connection_(std::move(connection)) {}
    std::size_t read(std::uint8_t* output, std::size_t capacity) override {
        return connection_.receive(output, capacity);
    }

private:
    transport::UdpConnection connection_;
};

}  // namespace

struct UnitreeL2Reader::Impl {
    Impl(
        std::unique_ptr<IUnitreeTransport> transport_value,
        std::chrono::milliseconds frame_timeout_value)
        : transport(std::move(transport_value)),
          frame_timeout(frame_timeout_value) {}

    RawPacket read_frame() {
        const auto deadline = std::chrono::steady_clock::now() + frame_timeout;
        for (;;) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error(
                    "Unitree L2 did not produce a valid frame within " +
                    std::to_string(frame_timeout.count()) + " ms");
            }
            const auto magic = std::search(
                bytes.begin(), bytes.end(), frame_magic.begin(), frame_magic.end());
            if (magic != bytes.begin()) {
                if (magic == bytes.end()) {
                    const auto keep = std::min<std::size_t>(
                        bytes.size(), frame_magic.size() - 1);
                    bytes.erase(bytes.begin(), bytes.end() - keep);
                } else {
                    bytes.erase(bytes.begin(), magic);
                }
            }

            if (bytes.size() >= unitree_protocol::frame_header_size) {
                const auto declared_size = unitree_protocol::read_u32_le(bytes, 8);
                if (declared_size < unitree_protocol::frame_header_size +
                                        unitree_protocol::frame_tail_size ||
                    declared_size > unitree_protocol::maximum_frame_size) {
                    bytes.erase(bytes.begin());
                    continue;
                }
                if (bytes.size() >= declared_size) {
                    std::vector<std::uint8_t> frame(
                        bytes.begin(), bytes.begin() + declared_size);
                    bytes.erase(bytes.begin(), bytes.begin() + declared_size);
                    try {
                        unitree_protocol::validate_frame(frame);
                        const auto timestamp =
                            unitree_protocol::packet_timestamp_ns(frame)
                                .value_or(system_timestamp_ns());
                        return RawPacket(std::move(frame), timestamp);
                    } catch (const std::exception&) {
                        // Continue scanning after a corrupted serial/UDP frame.
                        continue;
                    }
                }
            }

            std::array<std::uint8_t, read_buffer_size> input{};
            const auto count = transport->read(input.data(), input.size());
            if (count == 0) {
                throw std::runtime_error("Unitree L2 transport returned no data");
            }
            bytes.insert(bytes.end(), input.begin(), input.begin() + count);
            if (bytes.size() > unitree_protocol::maximum_frame_size * 2) {
                bytes.erase(
                    bytes.begin(),
                    bytes.end() - unitree_protocol::maximum_frame_size);
            }
        }
    }

    static std::uint64_t system_timestamp_ns() {
        const auto elapsed = std::chrono::system_clock::now().time_since_epoch();
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
    }

    std::unique_ptr<IUnitreeTransport> transport;
    std::vector<std::uint8_t> bytes;
    std::chrono::milliseconds frame_timeout;
};

namespace {

std::unique_ptr<UnitreeL2Reader> make_serial_reader(
    const UnitreeL2Config& config,
    std::chrono::milliseconds timeout) {
    auto connection = transport::SerialConnection::open(
        config.serial_port, config.baud_rate, timeout);
    return std::make_unique<UnitreeL2Reader>(
        std::make_unique<UnitreeL2Reader::Impl>(
            std::make_unique<SerialUnitreeTransport>(std::move(connection)),
            timeout));
}

std::unique_ptr<UnitreeL2Reader> make_udp_reader(
    const UnitreeL2Config& config,
    std::chrono::milliseconds timeout) {
    auto connection = transport::UdpConnection::connect(
        config.local_ip,
        config.local_port,
        config.sensor_ip,
        config.sensor_port,
        timeout);
    return std::make_unique<UnitreeL2Reader>(
        std::make_unique<UnitreeL2Reader::Impl>(
            std::make_unique<UdpUnitreeTransport>(std::move(connection)),
            timeout));
}

std::unique_ptr<UnitreeL2Reader> probe_reader(
    const std::string& label,
    std::function<std::unique_ptr<UnitreeL2Reader>()> create) {
    auto reader = create();
    auto first_packet = reader->read_raw_packet();
    reader->keep_for_first_read(std::move(first_packet));
    std::cout << "Unitree L2 transport selected: " << label << '\n';
    return reader;
}

}  // namespace

UnitreeL2Reader::UnitreeL2Reader(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

UnitreeL2Reader::~UnitreeL2Reader() = default;

RawPacket UnitreeL2Reader::read_raw_packet() {
    if (pending_packet_) {
        auto packet = std::move(*pending_packet_);
        pending_packet_.reset();
        return packet;
    }
    return implementation_->read_frame();
}

void UnitreeL2Reader::keep_for_first_read(RawPacket packet) {
    pending_packet_ = std::move(packet);
}

UnitreeL2Decoder::UnitreeL2Decoder(std::size_t cloud_scan_count)
    : cloud_scan_count_(cloud_scan_count) {
    if (cloud_scan_count_ == 0 || cloud_scan_count_ > 255) {
        throw std::invalid_argument(
            "Unitree cloud scan count must be between 1 and 255");
    }
    accumulated_points_.reserve(
        cloud_scan_count_ * unitree_protocol::points_per_packet);
}

models::PointCloudFrame UnitreeL2Decoder::decode_packet(
    const RawPacket& packet) {
    const auto& bytes = packet.bytes();
    unitree_protocol::validate_frame(bytes);
    if (unitree_protocol::packet_type(bytes) !=
        unitree_protocol::point_packet_type) {
        return {};
    }

    if (accumulated_scan_count_ == 0) {
        cloud_timestamp_ns_ = packet.timestamp_ns().value_or(0);
    }
    auto points = unitree_protocol::decode_point_packet(
        bytes, static_cast<std::uint8_t>(accumulated_scan_count_));
    accumulated_points_.insert(
        accumulated_points_.end(),
        std::make_move_iterator(points.begin()),
        std::make_move_iterator(points.end()));
    ++accumulated_scan_count_;

    if (accumulated_scan_count_ < cloud_scan_count_) {
        return {};
    }

    models::PointCloudFrame frame(
        cloud_timestamp_ns_, std::move(accumulated_points_));
    accumulated_scan_count_ = 0;
    cloud_timestamp_ns_ = 0;
    accumulated_points_.clear();
    accumulated_points_.reserve(
        cloud_scan_count_ * unitree_protocol::points_per_packet);
    return frame;
}

std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
UnitreeL2::connect(
    const UnitreeL2Config& config,
    std::chrono::milliseconds connection_timeout) {
    std::unique_ptr<UnitreeL2Reader> reader;
    if (config.connection_mode == UnitreeConnectionMode::serial) {
        reader = probe_reader("serial/USB", [&] {
            return make_serial_reader(config, connection_timeout);
        });
    } else if (config.connection_mode == UnitreeConnectionMode::udp) {
        reader = probe_reader("Ethernet/UDP", [&] {
            return make_udp_reader(config, connection_timeout);
        });
    } else {
        std::string serial_failure;
        try {
            reader = probe_reader("serial/USB", [&] {
                return make_serial_reader(config, connection_timeout);
            });
        } catch (const std::exception& error) {
            serial_failure = error.what();
        }
        if (!reader) {
            try {
                reader = probe_reader("Ethernet/UDP", [&] {
                    return make_udp_reader(config, connection_timeout);
                });
            } catch (const std::exception& error) {
                throw std::runtime_error(
                    "Unitree L2 auto-detection failed; serial: " +
                    serial_failure + "; UDP: " + error.what());
            }
        }
    }

    return {
        std::move(reader),
        std::make_unique<UnitreeL2Decoder>(
            unitree_protocol::default_cloud_scan_count),
    };
}

}  // namespace vista::devices
