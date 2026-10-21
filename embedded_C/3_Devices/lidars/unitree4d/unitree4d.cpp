#include "3_Devices/lidars/unitree4d/unitree4d.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>

#include "2_Transport/peripheral/serial/serial.hpp"
#include "2_Transport/peripheral/udp/udp.hpp"
#include "3_Devices/lidars/unitree4d/unitree_protocol.hpp"

namespace vista::devices {

namespace {

constexpr std::array<std::uint8_t, 4> frame_magic{0x55, 0xAA, 0x05, 0x0A};
constexpr std::size_t read_buffer_size = 65'536;

class SerialUnitreeTransport final : public IUnitreeTransport {
public:
    explicit SerialUnitreeTransport(transport::SerialConnection connection)
        : connection_(std::move(connection)) {}
    std::size_t read(std::uint8_t* output, std::size_t capacity) override {
        return connection_.read_some(output, capacity);
    }
    void write(const std::vector<std::uint8_t>& bytes) override {
        connection_.write_all(bytes.data(), bytes.size());
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
    void write(const std::vector<std::uint8_t>& bytes) override {
        connection_.send(bytes.data(), bytes.size());
    }

private:
    transport::UdpConnection connection_;
};

}  // namespace

struct UnitreeL2InitState {
    std::mutex mutex;
    bool mode_accepted{};
    bool restart_attempted{};
    bool boot_wait_complete{};
    std::uint32_t applied_mode{};
    UnitreeConnectionMode control_mode{UnitreeConnectionMode::serial};
};

std::shared_ptr<UnitreeL2InitState> make_unitree_l2_init_state() {
    return std::make_shared<UnitreeL2InitState>();
}

struct UnitreeL2Reader::Impl {
    Impl(
        std::unique_ptr<IUnitreeTransport> transport_value,
        std::chrono::milliseconds frame_timeout_value)
        : transport(std::move(transport_value)),
          frame_timeout(frame_timeout_value) {}

    RawPacket read_frame() {
        return read_frame_until(std::chrono::steady_clock::now() + frame_timeout);
    }

    RawPacket read_frame_until(std::chrono::steady_clock::time_point deadline) {
        for (;;) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error(
                    "Unitree L2 did not produce a valid frame within " +
                    std::to_string(frame_timeout.count()) + " ms");
            }
            const auto first = bytes.begin() + static_cast<std::ptrdiff_t>(read_offset);
            const auto magic = std::search(
                first, bytes.end(), frame_magic.begin(), frame_magic.end());
            if (magic != first) {
                if (magic == bytes.end()) {
                    const auto keep = std::min<std::size_t>(
                        bytes.size() - read_offset, frame_magic.size() - 1);
                    read_offset = bytes.size() - keep;
                } else {
                    read_offset = static_cast<std::size_t>(magic - bytes.begin());
                }
            }

            if (bytes.size() - read_offset >= unitree_protocol::frame_header_size) {
                const auto declared_size = unitree_protocol::read_u32_le(bytes, read_offset + 8);
                if (declared_size < unitree_protocol::frame_header_size +
                                        unitree_protocol::frame_tail_size ||
                    declared_size > unitree_protocol::maximum_frame_size) {
                    ++read_offset;
                    continue;
                }
                if (bytes.size() - read_offset >= declared_size) {
                    std::vector<std::uint8_t> frame(
                        bytes.begin() + static_cast<std::ptrdiff_t>(read_offset),
                        bytes.begin() + static_cast<std::ptrdiff_t>(read_offset + declared_size));
                    read_offset += declared_size;
                    try {
                        unitree_protocol::validate_frame(frame);
                        const auto timestamp =
                            unitree_protocol::packet_timestamp_ns(frame)
                                .value_or(system_timestamp_ns());
                        RawPacket packet(std::move(frame), timestamp);
                        packet.unitree_verification_.checked.store(true, std::memory_order_relaxed);
                        return packet;
                    } catch (const std::exception&) {
                        // Continue scanning after a corrupted serial/UDP frame.
                        continue;
                    }
                }
            }

            // Compact only when another transport read is needed, not for each
            // small IMU/point packet extracted from the same batch.
            if (read_offset) {
                bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(read_offset));
                read_offset = 0;
            }
            std::array<std::uint8_t, read_buffer_size> input;
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
    std::size_t read_offset{};
    std::chrono::milliseconds frame_timeout;
};

namespace {

UnitreeConnectionHooks hardware_hooks() {
    return {
        [](UnitreeConnectionMode mode, const UnitreeL2Config& config,
           std::chrono::milliseconds timeout) -> std::unique_ptr<IUnitreeTransport> {
            if (mode == UnitreeConnectionMode::serial) {
                return std::make_unique<SerialUnitreeTransport>(
                    transport::SerialConnection::open(
                        config.serial_port, config.baud_rate, timeout));
            }
            return std::make_unique<UdpUnitreeTransport>(
                transport::UdpConnection::connect(
                    config.local_ip, config.local_port, config.sensor_ip,
                    config.sensor_port, timeout));
        },
        [](std::chrono::milliseconds delay) { std::this_thread::sleep_for(delay); },
    };
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

void UnitreeL2Decoder::discard_pending_scan() {
    accumulated_scan_count_ = 0;
    cloud_timestamp_ns_ = 0;
    accumulated_points_.clear();
}

std::optional<models::ImuFrame> UnitreeL2Decoder::decode_imu_packet(
    const RawPacket& packet) {
    if (unitree_protocol::packet_type(packet.bytes()) !=
        unitree_protocol::imu_packet_type) {
        return std::nullopt;
    }
    return unitree_protocol::decode_imu_packet(validated_frame(packet));
}

unitree_protocol::ValidatedFrame UnitreeL2Decoder::validated_frame(const RawPacket& packet) const {
    // Moved-from packets may retain copied metadata but never certified bytes.
    if (packet.empty() || !packet.unitree_verification_.checked.load(std::memory_order_relaxed)) {
        unitree_protocol::validate_frame(packet.bytes());
        packet.unitree_verification_.checked.store(true, std::memory_order_relaxed);
    }
    return unitree_protocol::ValidatedFrame(packet.bytes(), 0);
}

models::PointCloudFrame UnitreeL2Decoder::decode_packet(
    const RawPacket& packet) {
    const auto& bytes = packet.bytes();
    const auto checked = validated_frame(packet);
    if (unitree_protocol::packet_type(bytes) !=
        unitree_protocol::point_packet_type) {
        return {};
    }

    if (accumulated_scan_count_ == 0) {
        cloud_timestamp_ns_ = packet.timestamp_ns().value_or(0);
    }
    unitree_protocol::decode_point_packet(
        checked, static_cast<std::uint8_t>(accumulated_scan_count_), accumulated_points_);
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
    return connect(config, connection_timeout, hardware_hooks());
}

std::pair<std::unique_ptr<ILidarReader>, std::unique_ptr<ILidarDecoder>>
UnitreeL2::connect(
    const UnitreeL2Config& config,
    std::chrono::milliseconds connection_timeout,
    const UnitreeConnectionHooks& hooks) {
    if (connection_timeout.count() <= 0 || !hooks.open || !hooks.wait)
        throw std::invalid_argument("invalid Unitree connection timeout/hooks");

    const auto create = [&](UnitreeConnectionMode mode) {
        auto transport = hooks.open(mode, config, connection_timeout);
        if (!transport) throw std::runtime_error("Unitree transport could not be opened");
        return std::make_unique<UnitreeL2Reader>(
            std::make_unique<UnitreeL2Reader::Impl>(
                std::move(transport), connection_timeout));
    };
    std::unique_ptr<UnitreeL2Reader> reader;
    if (config.initialize_on_connect) {
        const auto command = unitree_protocol::encode_work_mode(config.work_mode);
        if (config.serial_port.empty() || config.baud_rate == 0 ||
            !config.initialization_state)
            throw std::invalid_argument("Unitree UART initialization requires serial configuration/session");
        auto& state = *config.initialization_state;
        std::lock_guard<std::mutex> lock(state.mutex);
        if (state.mode_accepted && state.applied_mode != config.work_mode)
            throw std::invalid_argument("Unitree WorkMode changed during an initialized session; restart the application");

        const auto configure = [&](UnitreeConnectionMode mode) {
            auto control = create(mode);
            control->implementation_->transport->write(command);
            const auto deadline = std::chrono::steady_clock::now() + connection_timeout;
            for (;;) {
                const auto packet = control->implementation_->read_frame_until(deadline);
                const auto& bytes = packet.bytes();
                if (unitree_protocol::packet_type(bytes) ==
                        unitree_protocol::work_mode_readback_packet_type &&
                    bytes.size() == 28) {
                    if (unitree_protocol::read_u32_le(bytes, 12) != config.work_mode)
                        throw std::runtime_error("Unitree WorkMode readback does not match requested mode");
                    state.mode_accepted = true;
                    state.applied_mode = config.work_mode;
                    state.control_mode = mode;
                    return control;
                }
                if (unitree_protocol::packet_type(bytes) !=
                        unitree_protocol::acknowledgement_packet_type ||
                    bytes.size() != 40 ||
                    unitree_protocol::read_u32_le(bytes, 12) !=
                        unitree_protocol::work_mode_packet_type)
                    continue;
                const auto status = unitree_protocol::read_u32_le(bytes, 24);
                if (status != 1)
                    throw std::runtime_error("Unitree rejected WorkMode; ACK status " +
                                             std::to_string(status));
                state.mode_accepted = true;
                state.applied_mode = config.work_mode;
                state.control_mode = mode;
                return control;
            }
        };

        if (!state.mode_accepted) {
            if (config.connection_mode != UnitreeConnectionMode::automatic) {
                reader = configure(config.connection_mode);
            } else {
                std::string serial_failure;
                try { reader = configure(UnitreeConnectionMode::serial); }
                catch (const std::exception& error) { serial_failure = error.what(); }
                if (!reader) {
                    try { reader = configure(UnitreeConnectionMode::udp); }
                    catch (const std::exception& error) {
                        throw std::runtime_error(
                            "Unitree L2 initialization failed (no restart sent); serial: " +
                            serial_failure + "; UDP: " + error.what() +
                            ". RAW replay requires InitializeOnConnect: 0");
                    }
                }
            }
        }
        if (!state.restart_attempted) {
            if (!reader) reader = create(state.control_mode);
            // Allow the mode to be persisted before reset, as in SDK2 examples.
            hooks.wait(std::chrono::milliseconds(1'000));
            // Even a partial/failed write must not cause an endless reset loop.
            state.restart_attempted = true;
            try {
                reader->implementation_->transport->write(unitree_protocol::encode_restart());
            } catch (const std::exception& error) {
                throw std::runtime_error(
                    std::string("Unitree restart write uncertain; further retries use UART only: ") +
                    error.what());
            }
            std::cout << "Unitree L2 WorkMode " << config.work_mode
                      << " acknowledged; restart sent once; switching to UART\n";
        }
        if (!state.boot_wait_complete) {
            // Keep the control handle open while the short reset command drains.
            hooks.wait(std::chrono::milliseconds(2'000));
            state.boot_wait_complete = true;
        }
        reader.reset(); // Discard pre-restart bytes/ACKs and release the COM port.
        reader = create(UnitreeConnectionMode::serial);
        const auto deadline = std::chrono::steady_clock::now() + connection_timeout;
        for (;;) {
            auto first = reader->implementation_->read_frame_until(deadline);
            if (unitree_protocol::packet_type(first.bytes()) == unitree_protocol::point_packet_type) {
                reader->keep_for_first_read(std::move(first));
                break;
            }
        }
        std::cout << "Unitree L2 transport selected: serial/USB (UART)\n";
    } else if (config.connection_mode == UnitreeConnectionMode::serial) {
        reader = probe_reader("serial/USB", [&] {
            return create(UnitreeConnectionMode::serial);
        });
    } else if (config.connection_mode == UnitreeConnectionMode::udp) {
        reader = probe_reader("Ethernet/UDP", [&] {
            return create(UnitreeConnectionMode::udp);
        });
    } else {
        std::string serial_failure;
        try {
            reader = probe_reader("serial/USB", [&] {
                return create(UnitreeConnectionMode::serial);
            });
        } catch (const std::exception& error) {
            serial_failure = error.what();
        }
        if (!reader) {
            try {
                reader = probe_reader("Ethernet/UDP", [&] {
                    return create(UnitreeConnectionMode::udp);
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
