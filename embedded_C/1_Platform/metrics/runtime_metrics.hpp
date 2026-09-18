#pragma once

#include <cstddef>
#include <cstdint>

namespace vista::platform {

struct StorageMetricsSnapshot {
    std::uint64_t raw_bytes_written{};
    std::uint64_t raw_messages_written{};
    std::uint64_t pcd_points_written{};
    std::uint64_t pcd_frames_written{};
    std::uint64_t write_errors{};
};

void record_raw_write(std::size_t bytes) noexcept;
void record_pcd_write(std::size_t points) noexcept;
void record_storage_write_error() noexcept;
StorageMetricsSnapshot storage_metrics_snapshot() noexcept;

}  // namespace vista::platform
