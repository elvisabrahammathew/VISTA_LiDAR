#include "2_Transport/storage/local.hpp"

#include <iomanip>
#include <stdexcept>

namespace vista::transport {

namespace {

constexpr int pcd_count_field_width = 20;

void create_parent_directory(const std::filesystem::path& path) {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

}  // namespace

RawCaptureWriter::RawCaptureWriter(const std::filesystem::path& path) {
    create_parent_directory(path);
    output_.open(path, std::ios::binary | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("could not create RAW output: " + path.string());
    }
}

void RawCaptureWriter::write_bytes(const std::vector<std::uint8_t>& bytes) {
    output_.write(
        reinterpret_cast<const char*>(bytes.data()),
        static_cast<std::streamsize>(bytes.size()));
    if (!output_) {
        throw std::runtime_error("could not write RAW output");
    }
}

void RawCaptureWriter::finish() {
    output_.flush();
    if (!output_) {
        throw std::runtime_error("could not flush RAW output");
    }
}

PcdWriter::PcdWriter(const std::filesystem::path& output_path) {
    create_parent_directory(output_path);
    output_.open(
        output_path,
        std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("could not create PCD output: " + output_path.string());
    }

    output_ << "# .PCD v0.7 - Point Cloud Data file format\n"
            << "VERSION 0.7\n"
            << "FIELDS x y z intensity ring return timestamp\n"
            << "SIZE 4 4 4 1 1 1 8\n"
            << "TYPE F F F U U U F\n"
            << "COUNT 1 1 1 1 1 1 1\n"
            << "WIDTH ";
    width_value_position_ = output_.tellp();
    output_ << std::left << std::setw(pcd_count_field_width) << 0 << '\n'
            << "HEIGHT 1\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS ";
    points_value_position_ = output_.tellp();
    output_ << std::left << std::setw(pcd_count_field_width) << 0 << '\n'
            << "DATA ascii\n";
    output_.flush();
    if (!output_) {
        throw std::runtime_error("could not initialize PCD output");
    }
}

PcdWriter::~PcdWriter() {
    if (!finished_) {
        try {
            update_header();
        } catch (...) {
            // Destructors must not propagate storage errors.
        }
    }
}

void PcdWriter::write_frame(const models::PointCloudFrame& frame) {
    output_ << std::fixed;
    for (const auto& point : frame.points) {
        const auto timestamp_seconds =
            static_cast<double>(point.timestamp_ns) / 1'000'000'000.0;
        output_ << std::setprecision(6)
                << point.x << ' ' << point.y << ' ' << point.z << ' '
                << static_cast<unsigned int>(point.intensity) << ' '
                << static_cast<unsigned int>(point.ring) << ' '
                << static_cast<unsigned int>(point.return_id) << ' '
                << std::setprecision(9) << timestamp_seconds << '\n';
        ++point_count_;
    }
    if (!output_) {
        throw std::runtime_error("could not write PCD point data");
    }
    update_header();
}

void PcdWriter::update_header() {
    const auto data_end = output_.tellp();
    if (data_end == std::streampos(-1)) {
        throw std::runtime_error("could not locate the end of PCD output");
    }

    output_.seekp(width_value_position_);
    output_ << std::left << std::setw(pcd_count_field_width) << point_count_;
    output_.seekp(points_value_position_);
    output_ << std::left << std::setw(pcd_count_field_width) << point_count_;
    output_.seekp(data_end);
    output_.flush();
    if (!output_) {
        throw std::runtime_error("could not update PCD output header");
    }
}

std::uint64_t PcdWriter::finish() {
    update_header();
    finished_ = true;
    return point_count_;
}

}  // namespace vista::transport
