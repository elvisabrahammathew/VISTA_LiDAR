#include "1_Platform/metrics/runtime_metrics.hpp"

#include <atomic>

namespace vista::platform {

namespace {
std::atomic<std::uint64_t> raw_bytes_written{0};
std::atomic<std::uint64_t> raw_messages_written{0};
std::atomic<std::uint64_t> pcd_points_written{0};
std::atomic<std::uint64_t> pcd_frames_written{0};
std::atomic<std::uint64_t> storage_write_errors{0};
}  // namespace

void record_raw_write(std::size_t bytes) noexcept {
    raw_bytes_written.fetch_add(bytes, std::memory_order_relaxed);
    raw_messages_written.fetch_add(1, std::memory_order_relaxed);
}

void record_pcd_write(std::size_t points) noexcept {
    pcd_points_written.fetch_add(points, std::memory_order_relaxed);
    pcd_frames_written.fetch_add(1, std::memory_order_relaxed);
}

void record_storage_write_error() noexcept {
    storage_write_errors.fetch_add(1, std::memory_order_relaxed);
}

StorageMetricsSnapshot storage_metrics_snapshot() noexcept {
    return {
        raw_bytes_written.load(std::memory_order_relaxed),
        raw_messages_written.load(std::memory_order_relaxed),
        pcd_points_written.load(std::memory_order_relaxed),
        pcd_frames_written.load(std::memory_order_relaxed),
        storage_write_errors.load(std::memory_order_relaxed),
    };
}

}  // namespace vista::platform
