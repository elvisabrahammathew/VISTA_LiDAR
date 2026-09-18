#pragma once

#include <cstdint>
#include <string>

namespace vista::models {

struct SystemHealthTelemetry {
    std::uint64_t timestamp_ns{};
    double uptime_seconds{};
    double cpu_percent{};
    double memory_mb{};
    double memory_percent{};
    std::uint64_t worker_count{};
    std::uint64_t failed_workers{};
};

struct WorkerHealthTelemetry {
    std::uint64_t timestamp_ns{};
    std::string worker_name;
    std::uint8_t priority{};
    bool running{};
    bool failed{};
    double uptime_seconds{};
};

struct StorageHealthTelemetry {
    std::uint64_t timestamp_ns{};
    double disk_free_gb{};
    std::uint64_t raw_bytes_written{};
    std::uint64_t raw_messages_written{};
    std::uint64_t pcd_points_written{};
    std::uint64_t pcd_frames_written{};
    std::uint64_t write_errors{};
};

/// Values remain zero and available=false when the host exposes no sensors.
struct PowerThermalTelemetry {
    std::uint64_t timestamp_ns{};
    bool available{};
    double cpu_temperature_c{};
    double gpu_temperature_c{};
    double board_power_w{};
    double fan_rpm{};
};

}  // namespace vista::models
