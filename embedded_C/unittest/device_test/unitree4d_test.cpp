#include <cmath>
#include <algorithm>
#include <deque>
#include <cstdint>
#include <cstring>
#include <vector>

#include "3_Devices/lidars/unitree4d/unitree4d.hpp"
#include "3_Devices/lidars/unitree4d/unitree_protocol.hpp"
#include "unittest/test.hpp"

namespace {

void write_u16_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint16_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void write_u32_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    std::uint32_t value) {
    bytes[offset] = static_cast<std::uint8_t>(value);
    bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
    bytes[offset + 2] = static_cast<std::uint8_t>(value >> 16U);
    bytes[offset + 3] = static_cast<std::uint8_t>(value >> 24U);
}

void write_f32_le(
    std::vector<std::uint8_t>& bytes,
    std::size_t offset,
    float value) {
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    write_u32_le(bytes, offset, bits);
}

vista::devices::RawPacket one_meter_unitree_packet() {
    constexpr std::size_t packet_size = 1'044;
    constexpr std::size_t calibration_offset = 64;
    constexpr std::size_t line_offset = 96;
    constexpr std::size_t point_count_offset = 128;
    constexpr std::size_t ranges_offset = 132;
    constexpr std::size_t intensities_offset = 732;
    constexpr std::size_t tail_offset = packet_size - 12;

    std::vector<std::uint8_t> bytes(packet_size);
    bytes[0] = 0x55;
    bytes[1] = 0xAA;
    bytes[2] = 0x05;
    bytes[3] = 0x0A;
    write_u32_le(bytes, 4, vista::devices::unitree_protocol::point_packet_type);
    write_u32_le(bytes, 8, packet_size);
    write_u32_le(bytes, 12, 7);  // sequence
    write_u32_le(bytes, 16, packet_size - 24);
    write_u32_le(bytes, 20, 1);  // timestamp seconds
    write_u32_le(bytes, 24, 2);  // timestamp nanoseconds

    // xi=-pi/2 and zero alpha/theta produce one point on positive X.
    write_f32_le(bytes, calibration_offset + 20, -1.57079632679F);
    write_f32_le(bytes, calibration_offset + 28, 0.001F);
    write_f32_le(bytes, line_offset + 12, 0.0F);
    write_f32_le(bytes, line_offset + 16, 100.0F);
    write_f32_le(bytes, line_offset + 28, 0.00001F);
    write_u32_le(bytes, point_count_offset, 1);
    write_u16_le(bytes, ranges_offset, 1'000);
    bytes[intensities_offset] = 42;
    bytes[packet_size - 2] = 0x00;
    bytes[packet_size - 1] = 0xFF;
    write_u32_le(
        bytes,
        tail_offset,
        vista::devices::unitree_protocol::crc32(
            bytes.data() + vista::devices::unitree_protocol::frame_header_size,
            packet_size -
                vista::devices::unitree_protocol::frame_header_size -
                vista::devices::unitree_protocol::frame_tail_size));
    return vista::devices::RawPacket(std::move(bytes), 1'000'000'002ULL);
}

vista::devices::RawPacket unitree_imu_packet() {
    constexpr std::size_t packet_size = 80;
    constexpr std::size_t orientation_offset = 28;
    constexpr std::size_t angular_velocity_offset = 44;
    constexpr std::size_t linear_acceleration_offset = 56;
    constexpr std::size_t tail_offset = packet_size - 12;

    std::vector<std::uint8_t> bytes(packet_size);
    bytes[0] = 0x55;
    bytes[1] = 0xAA;
    bytes[2] = 0x05;
    bytes[3] = 0x0A;
    write_u32_le(bytes, 4, vista::devices::unitree_protocol::imu_packet_type);
    write_u32_le(bytes, 8, packet_size);
    write_u32_le(bytes, 12, 8);  // sequence
    write_u32_le(bytes, 16, packet_size - 24);
    write_u32_le(bytes, 20, 2);  // timestamp seconds
    write_u32_le(bytes, 24, 3);  // timestamp nanoseconds
    write_f32_le(bytes, orientation_offset, 0.1F);
    write_f32_le(bytes, orientation_offset + 4, 0.2F);
    write_f32_le(bytes, orientation_offset + 8, 0.3F);
    write_f32_le(bytes, orientation_offset + 12, 0.9F);
    write_f32_le(bytes, angular_velocity_offset, 1.1F);
    write_f32_le(bytes, angular_velocity_offset + 4, 1.2F);
    write_f32_le(bytes, angular_velocity_offset + 8, 1.3F);
    write_f32_le(bytes, linear_acceleration_offset, 9.7F);
    write_f32_le(bytes, linear_acceleration_offset + 4, 0.4F);
    write_f32_le(bytes, linear_acceleration_offset + 8, -0.5F);
    bytes[packet_size - 2] = 0x00;
    bytes[packet_size - 1] = 0xFF;
    write_u32_le(
        bytes,
        tail_offset,
        vista::devices::unitree_protocol::crc32(
            bytes.data() + vista::devices::unitree_protocol::frame_header_size,
            packet_size -
                vista::devices::unitree_protocol::frame_header_size -
                vista::devices::unitree_protocol::frame_tail_size));
    return vista::devices::RawPacket(std::move(bytes), 2'000'000'003ULL);
}

}  // namespace

VISTA_TEST(unitree_validates_and_decodes_one_meter_point) {
    const auto packet = one_meter_unitree_packet();
    vista::devices::unitree_protocol::validate_frame(packet.bytes());
    const auto points = vista::devices::unitree_protocol::decode_point_packet(
        packet.bytes(), 3);

    VISTA_CHECK(points.size() == 1);
    VISTA_CHECK_NEAR(points[0].x, 1.0F, 1.0e-5F);
    VISTA_CHECK_NEAR(points[0].y, 0.0F, 1.0e-5F);
    VISTA_CHECK_NEAR(points[0].z, 0.0F, 1.0e-5F);
    VISTA_CHECK(points[0].intensity == 42);
    VISTA_CHECK(points[0].ring == 3);
    VISTA_CHECK(points[0].timestamp_ns == 1'000'000'002ULL);
}

VISTA_TEST(unitree_decoder_emits_cloud_after_configured_scan_count) {
    vista::devices::UnitreeL2Decoder decoder(2);
    auto first = decoder.decode_packet(one_meter_unitree_packet());
    VISTA_CHECK(first.points.empty());

    auto second = decoder.decode_packet(one_meter_unitree_packet());
    VISTA_CHECK(second.points.size() == 2);
    VISTA_CHECK(second.points[0].ring == 0);
    VISTA_CHECK(second.points[1].ring == 1);
    VISTA_CHECK(second.timestamp_ns == 1'000'000'002ULL);
}

VISTA_TEST(unitree_decodes_imu_packet) {
    const auto packet = unitree_imu_packet();
    const auto sample =
        vista::devices::unitree_protocol::decode_imu_packet(packet.bytes());

    VISTA_CHECK(sample.timestamp_ns == 2'000'000'003ULL);
    VISTA_CHECK_NEAR(sample.orientation_x, 0.1F, 1.0e-6F);
    VISTA_CHECK_NEAR(sample.orientation_w, 0.9F, 1.0e-6F);
    VISTA_CHECK_NEAR(sample.angular_velocity_z_rad_s, 1.3F, 1.0e-6F);
    VISTA_CHECK_NEAR(sample.linear_acceleration_x_m_s2, 9.7F, 1.0e-6F);
}

VISTA_TEST(unitree_decoder_clock_epoch_discards_partial_scan_only) {
    vista::devices::UnitreeL2Decoder decoder(2);
    VISTA_CHECK(decoder.decode_packet(one_meter_unitree_packet()).points.empty());
    decoder.discard_pending_scan();
    VISTA_CHECK(decoder.decode_packet(one_meter_unitree_packet()).points.empty());
    auto cloud=decoder.decode_packet(one_meter_unitree_packet());
    VISTA_CHECK(cloud.points.size()==2 && cloud.points[0].ring==0 && cloud.points[1].ring==1);
    VISTA_CHECK(decoder.decode_imu_packet(unitree_imu_packet()));
}

VISTA_TEST(unitree_decoder_routes_imu_separately_from_pointcloud) {
    vista::devices::UnitreeL2Decoder decoder(2);
    const auto imu = decoder.decode_imu_packet(unitree_imu_packet());
    VISTA_CHECK(imu.has_value());
    VISTA_CHECK(decoder.decode_packet(unitree_imu_packet()).points.empty());
    VISTA_CHECK(!decoder.decode_imu_packet(one_meter_unitree_packet()).has_value());
}

VISTA_TEST(unitree_rejects_corrupted_crc) {
    auto packet = one_meter_unitree_packet();
    auto bytes = packet.bytes();
    bytes[200] ^= 0x01;
    VISTA_CHECK_THROWS(vista::devices::unitree_protocol::validate_frame(bytes));
}

namespace {
using vista::devices::UnitreeConnectionMode;
using vista::devices::UnitreeL2;
using vista::devices::UnitreeL2Config;
namespace protocol = vista::devices::unitree_protocol;
using Milliseconds = std::chrono::milliseconds;

std::vector<std::uint8_t> mode_ack(std::uint32_t status = 1,
                                  std::uint32_t type = protocol::work_mode_packet_type) {
    std::vector<std::uint8_t> bytes(40, 0);
    bytes[0] = 0x55; bytes[1] = 0xAA; bytes[2] = 0x05; bytes[3] = 0x0A;
    write_u32_le(bytes, 4, protocol::acknowledgement_packet_type);
    write_u32_le(bytes, 8, 40);
    write_u32_le(bytes, 12, type);
    write_u32_le(bytes, 24, status);
    write_u32_le(bytes, 28, protocol::crc32(bytes.data() + 12, 16));
    bytes.back() = 0xFF;
    return bytes;
}

struct OpenPlan {
    UnitreeConnectionMode mode;
    std::vector<std::uint8_t> input;
    bool fail_open{};
    bool fail_restart{};
    bool repeat{};
    std::size_t chunk{65'536};
};

struct UnitreeHarness {
    struct FakeTransport final : vista::devices::IUnitreeTransport {
        FakeTransport(UnitreeHarness& owner, OpenPlan plan)
            : owner(owner), plan(std::move(plan)) { ++owner.active; }
        ~FakeTransport() override { --owner.active; }
        std::size_t read(std::uint8_t* out, std::size_t capacity) override {
            if (cursor == plan.input.size() && plan.repeat) cursor = 0;
            if (cursor == plan.input.size()) throw std::runtime_error("fake timeout");
            const auto count = std::min({capacity, plan.chunk, plan.input.size() - cursor});
            std::copy_n(plan.input.begin() + cursor, count, out);
            cursor += count;
            return count;
        }
        void write(const std::vector<std::uint8_t>& bytes) override {
            owner.writes.push_back(bytes);
            if (plan.fail_restart && protocol::packet_type(bytes) == protocol::user_command_packet_type)
                throw std::runtime_error("fake partial restart write");
        }
        UnitreeHarness& owner;
        OpenPlan plan;
        std::size_t cursor{};
    };
    std::deque<OpenPlan> plans;
    std::vector<UnitreeConnectionMode> opens;
    std::vector<std::vector<std::uint8_t>> writes;
    std::vector<Milliseconds> waits;
    int active{};
    vista::devices::UnitreeConnectionHooks hooks() {
        return {
            [this](UnitreeConnectionMode mode, const UnitreeL2Config&, Milliseconds)
                -> std::unique_ptr<vista::devices::IUnitreeTransport> {
                VISTA_CHECK(active == 0); // The control COM must close before reopen.
                VISTA_CHECK(!plans.empty());
                auto plan = std::move(plans.front()); plans.pop_front();
                opens.push_back(mode);
                VISTA_CHECK(plan.mode == mode);
                if (plan.fail_open) throw std::runtime_error("fake unavailable port");
                return std::make_unique<FakeTransport>(*this, std::move(plan));
            },
            [this](Milliseconds delay) { waits.push_back(delay); },
        };
    }
};

UnitreeL2Config serial_config() {
    UnitreeL2Config config;
    config.connection_mode = UnitreeConnectionMode::serial;
    config.serial_port = "FAKE-COM";
    return config;
}
} // namespace

VISTA_TEST(unitree_command_bytes_match_sdk2_v2_0_10_capture) {
    // Independent SDK loopback capture: header, payload and CRC only; SDK
    // leaves the tail's message-check/reserved bytes uninitialized.
    const auto mode = protocol::encode_work_mode(9);
    const std::vector<std::uint8_t> prefix{
        0x55,0xAA,0x05,0x0A,0xD2,0x07,0,0,0x1C,0,0,0,
        9,0,0,0,0x96,0x90,0x4C,0x5C};
    VISTA_CHECK(mode.size() == 28);
    VISTA_CHECK(std::equal(prefix.begin(), prefix.end(), mode.begin()));
    const auto reset = protocol::encode_restart();
    const std::vector<std::uint8_t> reset_prefix{
        0x55,0xAA,0x05,0x0A,0x64,0,0,0,0x20,0,0,0,
        1,0,0,0,1,0,0,0,0x92,0xB8,0x34,0x11};
    VISTA_CHECK(reset.size() == 32);
    VISTA_CHECK(std::equal(reset_prefix.begin(), reset_prefix.end(), reset.begin()));
    protocol::validate_frame(mode);
    protocol::validate_frame(reset);
    for (std::size_t i = 20; i < 27; ++i) VISTA_CHECK(mode[i] == 0);
}

VISTA_TEST(unitree_command_rejects_enet_and_reserved_mode_bits) {
    VISTA_CHECK_THROWS(protocol::encode_work_mode(1));
    VISTA_CHECK_THROWS(protocol::encode_work_mode(0));
    VISTA_CHECK_THROWS(protocol::encode_work_mode(41));
    VISTA_CHECK_THROWS(protocol::encode_work_mode(11)); // 2D is not decoded.
    VISTA_CHECK_THROWS(protocol::encode_work_mode(25)); // Standby never streams.
    protocol::validate_frame(protocol::encode_work_mode(8));
}

VISTA_TEST(unitree_serial_init_sets_mode_then_restart_and_preserves_first_new_point) {
    auto config = serial_config();
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack(), false, false, false, 3});
    auto post_boot = unitree_imu_packet().bytes();
    const auto point = one_meter_unitree_packet().bytes();
    post_boot.insert(post_boot.end(), point.begin(), point.end());
    h.plans.push_back({UnitreeConnectionMode::serial, post_boot});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
    VISTA_CHECK(h.writes[0] == protocol::encode_work_mode(9));
    VISTA_CHECK(h.writes[1] == protocol::encode_restart());
    VISTA_CHECK(h.waits == std::vector<Milliseconds>({Milliseconds(1000), Milliseconds(2000)}));
    VISTA_CHECK(parts.first->read_raw_packet().bytes() == point);
}

VISTA_TEST(unitree_udp_initialization_always_hands_off_to_uart) {
    auto config = serial_config(); config.connection_mode = UnitreeConnectionMode::udp;
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::udp, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.opens == std::vector<UnitreeConnectionMode>({
        UnitreeConnectionMode::udp, UnitreeConnectionMode::serial}));
    VISTA_CHECK(h.writes.size() == 2);
}

VISTA_TEST(unitree_auto_initialization_tries_udp_after_serial_without_ack) {
    auto config = serial_config(); config.connection_mode = UnitreeConnectionMode::automatic;
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, {}});
    h.plans.push_back({UnitreeConnectionMode::udp, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.opens.size() == 3);
    VISTA_CHECK(h.writes.size() == 3); // Two mode attempts; exactly one restart.
    VISTA_CHECK(h.writes.back() == protocol::encode_restart());
}

VISTA_TEST(unitree_reconnect_config_copies_do_not_repeat_reset_or_mode) {
    auto config = serial_config();
    const auto copy = config;
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    { auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks()); }
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(copy, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
    VISTA_CHECK(h.waits.size() == 2);
}

VISTA_TEST(unitree_uart_unavailable_after_reset_retries_uart_without_reset_loop) {
    auto config = serial_config(); config.connection_mode = UnitreeConnectionMode::udp;
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::udp, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, {}, true});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
    VISTA_CHECK(h.opens.back() == UnitreeConnectionMode::serial);
}

VISTA_TEST(unitree_partial_restart_write_never_triggers_another_restart) {
    auto config = serial_config();
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack(), false, true});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
    VISTA_CHECK(h.waits.size() == 2);
}

VISTA_TEST(unitree_rejected_mode_ack_does_not_restart) {
    auto config = serial_config(); UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack(2)});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    VISTA_CHECK(h.writes.size() == 1);
    VISTA_CHECK(h.waits.empty());
}

VISTA_TEST(unitree_exact_mode_readback_also_confirms_configuration) {
    auto config = serial_config(); UnitreeHarness h;
    auto readback = protocol::encode_work_mode(9);
    write_u32_le(readback, 4, protocol::work_mode_readback_packet_type);
    h.plans.push_back({UnitreeConnectionMode::serial, readback});
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
}

VISTA_TEST(unitree_mismatched_mode_readback_never_restarts) {
    auto config = serial_config(); UnitreeHarness h;
    auto readback = protocol::encode_work_mode(8);
    write_u32_le(readback, 4, protocol::work_mode_readback_packet_type);
    h.plans.push_back({UnitreeConnectionMode::serial, readback});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    VISTA_CHECK(h.writes.size() == 1);
}

VISTA_TEST(unitree_mode_cannot_change_in_a_live_initialization_session) {
    auto config = serial_config(); UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    { auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks()); }
    config.work_mode = 8;
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    VISTA_CHECK(h.writes.size() == 2);
}

VISTA_TEST(unitree_acknowledged_mode_survives_interruption_before_reset) {
    auto config = serial_config(); UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack()});
    auto interrupted = h.hooks();
    interrupted.wait = [](Milliseconds) { throw std::runtime_error("interrupted persistence wait"); };
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), interrupted));
    h.plans.push_back({UnitreeConnectionMode::serial, {}}); // Reopen control, reset only.
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
    VISTA_CHECK(h.writes.back() == protocol::encode_restart());
}

VISTA_TEST(unitree_post_restart_imu_only_stream_does_not_confirm_3d_acquisition) {
    auto config = serial_config(); UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, mode_ack()});
    h.plans.push_back({UnitreeConnectionMode::serial, unitree_imu_packet().bytes(),
                       false, false, true});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(5), h.hooks()));
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.writes.size() == 2);
}

VISTA_TEST(unitree_unrelated_and_corrupt_ack_cannot_authorize_restart) {
    auto config = serial_config(); UnitreeHarness h;
    auto input = mode_ack(1, 100);
    auto corrupt = mode_ack(); corrupt[24] ^= 1;
    input.insert(input.end(), corrupt.begin(), corrupt.end());
    h.plans.push_back({UnitreeConnectionMode::serial, input});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    VISTA_CHECK(h.writes.size() == 1);
}

VISTA_TEST(unitree_continuous_points_cannot_extend_ack_deadline) {
    auto config = serial_config(); UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes(),
                       false, false, true});
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(5), h.hooks()));
    VISTA_CHECK(h.writes.size() == 1);
}

VISTA_TEST(unitree_passive_raw_replay_never_writes_or_waits) {
    auto config = serial_config(); config.initialize_on_connect = false;
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::serial, unitree_imu_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(parts.first->read_raw_packet().bytes() == unitree_imu_packet().bytes());
    VISTA_CHECK(h.writes.empty()); VISTA_CHECK(h.waits.empty());
}

VISTA_TEST(unitree_passive_udp_remains_udp_without_uart_or_commands) {
    auto config = serial_config(); config.initialize_on_connect = false;
    config.connection_mode = UnitreeConnectionMode::udp; config.serial_port.clear();
    UnitreeHarness h;
    h.plans.push_back({UnitreeConnectionMode::udp, one_meter_unitree_packet().bytes()});
    auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    VISTA_CHECK(h.opens.size() == 1); VISTA_CHECK(h.writes.empty());
}

VISTA_TEST(unitree_independent_sessions_each_initialize_once) {
    UnitreeHarness h;
    for (int session = 0; session < 2; ++session) {
        auto config = serial_config();
        h.plans.push_back({UnitreeConnectionMode::serial, mode_ack()});
        h.plans.push_back({UnitreeConnectionMode::serial, one_meter_unitree_packet().bytes()});
        auto parts = UnitreeL2::connect(config, Milliseconds(100), h.hooks());
    }
    VISTA_CHECK(h.writes.size() == 4);
}

VISTA_TEST(unitree_invalid_init_config_does_not_open_any_transport) {
    UnitreeHarness h; auto config = serial_config(); config.work_mode = 1;
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    config.work_mode = 9; config.serial_port.clear();
    VISTA_CHECK_THROWS(UnitreeL2::connect(config, Milliseconds(100), h.hooks()));
    VISTA_CHECK(h.opens.empty());
}
