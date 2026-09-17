#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

#include "models/lidars/pointcloud.hpp"

namespace vista::transport {

class RawCaptureWriter {
public:
    explicit RawCaptureWriter(const std::filesystem::path& path);
    void write_bytes(const std::vector<std::uint8_t>& bytes);
    void finish();

private:
    std::ofstream output_;
};

class PcdWriter {
public:
    explicit PcdWriter(const std::filesystem::path& output_path);
    ~PcdWriter();

    PcdWriter(const PcdWriter&) = delete;
    PcdWriter& operator=(const PcdWriter&) = delete;

    void write_frame(const models::PointCloudFrame& frame);
    std::uint64_t finish();

private:
    /// Updates the fixed-width WIDTH/POINTS fields and flushes live PCD data.
    void update_header();

    std::fstream output_;
    std::streampos width_value_position_{};
    std::streampos points_value_position_{};
    std::uint64_t point_count_{};
    bool finished_{false};
};

}  // namespace vista::transport
