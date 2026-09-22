#include "3_Devices/lidars/unitree4d/unitree_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

namespace vista::devices::unitree_protocol {

namespace {

constexpr std::uint8_t header_magic[]{0x55, 0xAA, 0x05, 0x0A};
constexpr std::size_t data_info_offset = frame_header_size;
constexpr std::size_t timestamp_seconds_offset = data_info_offset + 8;
constexpr std::size_t timestamp_nanoseconds_offset = data_info_offset + 12;
constexpr std::size_t current_inside_state_size = 36;
constexpr std::size_t legacy_inside_state_size = 28;
constexpr std::size_t calibration_size = 32;
constexpr std::size_t line_data_size = 32;
constexpr std::size_t current_point_packet_size = 1'044;
constexpr std::size_t legacy_point_packet_size = 1'036;
constexpr std::size_t imu_orientation_offset = data_info_offset + 16;
constexpr std::size_t imu_angular_velocity_offset =
    imu_orientation_offset + 4 * sizeof(float);
constexpr std::size_t imu_linear_acceleration_offset =
    imu_angular_velocity_offset + 3 * sizeof(float);
constexpr std::size_t minimum_imu_packet_size =
    imu_linear_acceleration_offset + 3 * sizeof(float) + frame_tail_size;

std::uint16_t read_u16_le(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset) {
    if (offset + 2 > bytes.size()) {
        throw std::runtime_error("truncated Unitree uint16 field");
    }
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(bytes[offset]) |
        (static_cast<std::uint16_t>(bytes[offset + 1]) << 8U));
}

std::size_t inside_state_size(const std::vector<std::uint8_t>& frame) {
    if (frame.size() == current_point_packet_size) {
        return current_inside_state_size;
    }
    if (frame.size() == legacy_point_packet_size) {
        return legacy_inside_state_size;
    }
    throw std::runtime_error(
        "unsupported Unitree 3D point packet size " +
        std::to_string(frame.size()));
}

std::uint64_t seconds_to_nanoseconds(double seconds) {
    if (seconds <= 0.0) {
        return 0;
    }
    const auto value = seconds * 1.0e9;
    if (value >= static_cast<double>(
                     std::numeric_limits<std::uint64_t>::max())) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(value);
}

}  // namespace

std::uint32_t read_u32_le(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset) {
    if (offset + 4 > bytes.size()) {
        throw std::runtime_error("truncated Unitree uint32 field");
    }
    return static_cast<std::uint32_t>(bytes[offset]) |
           (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

float read_f32_le(
    const std::vector<std::uint8_t>& bytes,
    std::size_t offset) {
    const auto bits = read_u32_le(bytes, offset);
    float value{};
    static_assert(sizeof(value) == sizeof(bits), "float must be 32-bit");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFU;
    while (size-- > 0) {
        crc ^= *data++;
        for (std::uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U
                      ? (crc >> 1U) ^ 0xEDB88320U
                      : crc >> 1U;
        }
    }
    return ~crc;
}

std::uint32_t packet_type(const std::vector<std::uint8_t>& frame) {
    if (frame.size() < frame_header_size) {
        throw std::runtime_error("truncated Unitree frame header");
    }
    return read_u32_le(frame, 4);
}

std::optional<std::uint64_t> packet_timestamp_ns(
    const std::vector<std::uint8_t>& frame) {
    const auto type = packet_type(frame);
    if (type != point_packet_type && type != point_2d_packet_type &&
        type != imu_packet_type) {
        return std::nullopt;
    }
    if (frame.size() < timestamp_nanoseconds_offset + 4) {
        throw std::runtime_error("truncated Unitree timestamp");
    }
    const auto seconds =
        static_cast<std::uint64_t>(read_u32_le(frame, timestamp_seconds_offset));
    const auto nanoseconds = static_cast<std::uint64_t>(
        read_u32_le(frame, timestamp_nanoseconds_offset));
    if (nanoseconds >= 1'000'000'000ULL) {
        throw std::runtime_error("invalid Unitree nanosecond timestamp");
    }
    return seconds * 1'000'000'000ULL + nanoseconds;
}

void validate_frame(const std::vector<std::uint8_t>& frame) {
    if (frame.size() < frame_header_size + frame_tail_size) {
        throw std::runtime_error("truncated Unitree frame");
    }
    for (std::size_t index = 0; index < sizeof(header_magic); ++index) {
        if (frame[index] != header_magic[index]) {
            throw std::runtime_error("invalid Unitree frame header");
        }
    }
    const auto declared_size = read_u32_le(frame, 8);
    if (declared_size != frame.size() || declared_size > maximum_frame_size) {
        throw std::runtime_error("invalid Unitree declared frame size");
    }
    if (frame[frame.size() - 2] != 0x00 ||
        frame[frame.size() - 1] != 0xFF) {
        throw std::runtime_error("invalid Unitree frame tail");
    }
    const auto expected_crc = read_u32_le(frame, frame.size() - frame_tail_size);
    // Unitree stores CRC32 for the packet data only. The 12-byte frame header
    // and 12-byte frame tail are deliberately excluded.
    const auto actual_crc = crc32(
        frame.data() + frame_header_size,
        frame.size() - frame_header_size - frame_tail_size);
    if (expected_crc != actual_crc) {
        throw std::runtime_error("invalid Unitree frame CRC32");
    }
}

std::vector<models::PointXYZIRT> decode_point_packet(
    const std::vector<std::uint8_t>& frame,
    std::uint8_t ring) {
    validate_frame(frame);
    if (packet_type(frame) != point_packet_type) {
        throw std::invalid_argument("Unitree frame is not a 3D point packet");
    }

    const auto state_size = inside_state_size(frame);
    const auto calibration_offset = data_info_offset + 16 + state_size;
    const auto line_offset = calibration_offset + calibration_size;
    const auto point_count_offset = line_offset + line_data_size;
    const auto ranges_offset = point_count_offset + 4;
    const auto intensities_offset = ranges_offset + points_per_packet * 2;
    if (intensities_offset + points_per_packet + frame_tail_size !=
        frame.size()) {
        throw std::runtime_error("invalid Unitree point packet layout");
    }

    const auto point_count = read_u32_le(frame, point_count_offset);
    if (point_count > points_per_packet) {
        throw std::runtime_error("invalid Unitree point count");
    }

    const auto a_axis_dist = read_f32_le(frame, calibration_offset);
    const auto b_axis_dist = read_f32_le(frame, calibration_offset + 4);
    const auto theta_bias = read_f32_le(frame, calibration_offset + 8);
    const auto alpha_bias = read_f32_le(frame, calibration_offset + 12);
    const auto beta = read_f32_le(frame, calibration_offset + 16);
    const auto xi = read_f32_le(frame, calibration_offset + 20);
    const auto range_bias = read_f32_le(frame, calibration_offset + 24);
    const auto range_scale = read_f32_le(frame, calibration_offset + 28);

    const auto theta_start = read_f32_le(frame, line_offset);
    const auto theta_step = read_f32_le(frame, line_offset + 4);
    const auto range_min = read_f32_le(frame, line_offset + 12);
    const auto range_max = read_f32_le(frame, line_offset + 16);
    const auto alpha_start = read_f32_le(frame, line_offset + 20);
    const auto alpha_step = read_f32_le(frame, line_offset + 24);
    const auto time_step = read_f32_le(frame, line_offset + 28);

    const auto sin_beta = std::sin(beta);
    const auto cos_beta = std::cos(beta);
    const auto sin_xi = std::sin(xi);
    const auto cos_xi = std::cos(xi);
    const auto cos_beta_sin_xi = cos_beta * sin_xi;
    const auto sin_beta_cos_xi = sin_beta * cos_xi;
    const auto sin_beta_sin_xi = sin_beta * sin_xi;
    const auto cos_beta_cos_xi = cos_beta * cos_xi;
    const auto base_timestamp = packet_timestamp_ns(frame).value_or(0);

    std::vector<models::PointXYZIRT> points;
    points.reserve(point_count);
    for (std::uint32_t index = 0; index < point_count; ++index) {
        const auto raw_range = read_u16_le(frame, ranges_offset + index * 2);
        if (raw_range < 1) {
            continue;
        }
        const auto range = range_scale *
                           (static_cast<float>(raw_range) + range_bias);
        if (!std::isfinite(range) || range < range_min || range > range_max) {
            continue;
        }

        const auto alpha = alpha_start + alpha_bias +
                           alpha_step * static_cast<float>(index);
        const auto theta = theta_start + theta_bias +
                           theta_step * static_cast<float>(index);
        const auto sin_alpha = std::sin(alpha);
        const auto cos_alpha = std::cos(alpha);
        const auto sin_theta = std::sin(theta);
        const auto cos_theta = std::cos(theta);
        const auto a = (-cos_beta_sin_xi +
                        sin_beta_cos_xi * sin_alpha) *
                           range +
                       b_axis_dist;
        const auto b = cos_alpha * cos_xi * range;
        const auto c = (sin_beta_sin_xi +
                        cos_beta_cos_xi * sin_alpha) *
                       range;

        points.push_back(models::PointXYZIRT{
            cos_theta * a - sin_theta * b,
            sin_theta * a + cos_theta * b,
            c + a_axis_dist,
            frame[intensities_offset + index],
            ring,
            0,
            base_timestamp + seconds_to_nanoseconds(
                                 static_cast<double>(time_step) * index),
        });
    }
    return points;
}

models::ImuFrame decode_imu_packet(
    const std::vector<std::uint8_t>& frame) {
    validate_frame(frame);
    if (packet_type(frame) != imu_packet_type) {
        throw std::invalid_argument("Unitree frame is not an IMU packet");
    }
    // The documented fields occupy 80 bytes. Accept a larger valid packet as
    // well so reserved fields added by firmware do not break this decoder.
    if (frame.size() < minimum_imu_packet_size) {
        throw std::runtime_error("truncated Unitree IMU packet");
    }

    models::ImuFrame sample;
    sample.timestamp_ns = packet_timestamp_ns(frame).value_or(0);
    sample.orientation_x = read_f32_le(frame, imu_orientation_offset);
    sample.orientation_y = read_f32_le(frame, imu_orientation_offset + 4);
    sample.orientation_z = read_f32_le(frame, imu_orientation_offset + 8);
    sample.orientation_w = read_f32_le(frame, imu_orientation_offset + 12);
    sample.angular_velocity_x_rad_s =
        read_f32_le(frame, imu_angular_velocity_offset);
    sample.angular_velocity_y_rad_s =
        read_f32_le(frame, imu_angular_velocity_offset + 4);
    sample.angular_velocity_z_rad_s =
        read_f32_le(frame, imu_angular_velocity_offset + 8);
    sample.linear_acceleration_x_m_s2 =
        read_f32_le(frame, imu_linear_acceleration_offset);
    sample.linear_acceleration_y_m_s2 =
        read_f32_le(frame, imu_linear_acceleration_offset + 4);
    sample.linear_acceleration_z_m_s2 =
        read_f32_le(frame, imu_linear_acceleration_offset + 8);

    const float values[]{
        sample.orientation_x,
        sample.orientation_y,
        sample.orientation_z,
        sample.orientation_w,
        sample.angular_velocity_x_rad_s,
        sample.angular_velocity_y_rad_s,
        sample.angular_velocity_z_rad_s,
        sample.linear_acceleration_x_m_s2,
        sample.linear_acceleration_y_m_s2,
        sample.linear_acceleration_z_m_s2,
    };
    if (!std::all_of(std::begin(values), std::end(values), [](float value) {
            return std::isfinite(value);
        })) {
        throw std::runtime_error("Unitree IMU packet contains a non-finite value");
    }
    return sample;
}

}  // namespace vista::devices::unitree_protocol
