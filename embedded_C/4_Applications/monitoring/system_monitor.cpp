#include "4_Applications/monitoring/system_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <psapi.h>
#endif

#include "1_Platform/metrics/runtime_metrics.hpp"
#include "models/telemetry.hpp"
#include "models/topics.hpp"

namespace vista::application {

namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t system_timestamp_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string current_exception_message() {
    try {
        throw;
    } catch (const std::exception& error) {
        return error.what();
    } catch (...) {
        return "unknown system-monitor failure";
    }
}

void interruptible_wait(
    Clock::time_point deadline,
    const platform::StopToken& stop) {
    while (!stop.is_stop_requested() && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

#ifdef _WIN32
std::uint64_t file_time_value(const FILETIME& value) {
    ULARGE_INTEGER integer{};
    integer.LowPart = value.dwLowDateTime;
    integer.HighPart = value.dwHighDateTime;
    return integer.QuadPart;
}
#endif

class CpuSampler {
public:
    double sample() {
        std::uint64_t idle{};
        std::uint64_t total{};
#ifdef _WIN32
        FILETIME idle_time{}, kernel_time{}, user_time{};
        if (GetSystemTimes(&idle_time, &kernel_time, &user_time) == 0) {
            return 0.0;
        }
        idle = file_time_value(idle_time);
        total = file_time_value(kernel_time) + file_time_value(user_time);
#else
        std::ifstream input("/proc/stat");
        std::string cpu;
        std::uint64_t user{}, nice{}, system{}, idle_value{}, io_wait{};
        std::uint64_t irq{}, soft_irq{}, steal{};
        if (!(input >> cpu >> user >> nice >> system >> idle_value >> io_wait >>
              irq >> soft_irq >> steal) || cpu != "cpu") {
            return 0.0;
        }
        idle = idle_value + io_wait;
        total = user + nice + system + idle_value + io_wait + irq + soft_irq + steal;
#endif
        double percent = 0.0;
        if (initialized_ && total > previous_total_) {
            const auto total_delta = total - previous_total_;
            const auto idle_delta = idle - previous_idle_;
            percent = 100.0 * static_cast<double>(total_delta - idle_delta) /
                      static_cast<double>(total_delta);
        }
        previous_idle_ = idle;
        previous_total_ = total;
        initialized_ = true;
        return std::clamp(percent, 0.0, 100.0);
    }

private:
    bool initialized_{};
    std::uint64_t previous_idle_{};
    std::uint64_t previous_total_{};
};

double process_memory_mb() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX counters{};
    if (GetProcessMemoryInfo(
            GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
            sizeof(counters)) == 0) {
        return 0.0;
    }
    return static_cast<double>(counters.WorkingSetSize) / (1024.0 * 1024.0);
#else
    std::ifstream input("/proc/self/status");
    std::string key;
    while (input >> key) {
        if (key == "VmRSS:") {
            std::uint64_t kib{};
            input >> kib;
            return static_cast<double>(kib) / 1024.0;
        }
        std::string rest;
        std::getline(input, rest);
    }
    return 0.0;
#endif
}

double system_memory_percent() {
#ifdef _WIN32
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    return GlobalMemoryStatusEx(&status) != 0
               ? static_cast<double>(status.dwMemoryLoad)
               : 0.0;
#else
    std::ifstream input("/proc/meminfo");
    std::string key;
    std::uint64_t total{}, available{};
    while (input >> key) {
        std::uint64_t value{};
        input >> value;
        if (key == "MemTotal:") {
            total = value;
        } else if (key == "MemAvailable:") {
            available = value;
        }
        std::string rest;
        std::getline(input, rest);
    }
    return total == 0 ? 0.0
                      : 100.0 * static_cast<double>(total - available) /
                            static_cast<double>(total);
#endif
}

double disk_free_gb(const std::filesystem::path& path) {
    std::error_code error;
    const auto space = std::filesystem::space(path, error);
    return error ? 0.0
                 : static_cast<double>(space.available) /
                       (1024.0 * 1024.0 * 1024.0);
}

#ifndef _WIN32
bool read_number(const std::filesystem::path& path, double& value) {
    std::ifstream input(path);
    return static_cast<bool>(input >> value);
}

models::PowerThermalTelemetry power_thermal_sample(std::uint64_t timestamp) {
    models::PowerThermalTelemetry result;
    result.timestamp_ns = timestamp;
    std::error_code error;
    const std::filesystem::path thermal_root("/sys/class/thermal");
    for (std::filesystem::directory_iterator iterator(thermal_root, error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        const auto directory = iterator->path();
        if (directory.filename().string().rfind("thermal_zone", 0) != 0) {
            continue;
        }
        double raw_temperature{};
        if (!read_number(directory / "temp", raw_temperature)) {
            continue;
        }
        std::ifstream type_input(directory / "type");
        std::string type;
        std::getline(type_input, type);
        const auto temperature = raw_temperature > 1'000.0
                                     ? raw_temperature / 1'000.0
                                     : raw_temperature;
        if (type.find("gpu") != std::string::npos ||
            type.find("GPU") != std::string::npos) {
            result.gpu_temperature_c =
                std::max(result.gpu_temperature_c, temperature);
        } else {
            result.cpu_temperature_c =
                std::max(result.cpu_temperature_c, temperature);
        }
        result.available = true;
    }

    const std::filesystem::path hwmon_root("/sys/class/hwmon");
    error.clear();
    for (std::filesystem::directory_iterator iterator(hwmon_root, error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        std::error_code child_error;
        for (std::filesystem::directory_iterator child(iterator->path(), child_error), child_end;
             !child_error && child != child_end;
             child.increment(child_error)) {
            const auto name = child->path().filename().string();
            double raw{};
            if (name.rfind("power", 0) == 0 &&
                name.find("_input") != std::string::npos &&
                read_number(child->path(), raw)) {
                result.board_power_w += raw / 1'000'000.0;
                result.available = true;
            } else if (name.rfind("fan", 0) == 0 &&
                       name.find("_input") != std::string::npos &&
                       read_number(child->path(), raw)) {
                result.fan_rpm = std::max(result.fan_rpm, raw);
                result.available = true;
            }
        }
    }
    return result;
}
#else
models::PowerThermalTelemetry power_thermal_sample(std::uint64_t timestamp) {
    models::PowerThermalTelemetry result;
    result.timestamp_ns = timestamp;
    return result;
}
#endif

SystemMonitorReport run_system_monitor(
    platform::TopicPublisher<models::SystemHealthTelemetry> system_publisher,
    platform::TopicPublisher<models::WorkerHealthTelemetry> worker_publisher,
    platform::TopicPublisher<models::StorageHealthTelemetry> storage_publisher,
    platform::TopicPublisher<models::PowerThermalTelemetry> power_publisher,
    const SystemMonitorConfig& config,
    const platform::StopToken& stop) {
    SystemMonitorReport report;
    CpuSampler cpu;
    const auto started = Clock::now();
    auto next_sample = started;

    while (!stop.is_stop_requested()) {
        const auto timestamp = system_timestamp_ns();
        const auto workers = platform::worker_runtime_snapshot();
        std::uint64_t running_workers{};
        std::uint64_t failed_workers{};
        for (const auto& worker : workers) {
            running_workers += worker.running ? 1U : 0U;
            failed_workers += worker.failed ? 1U : 0U;
            worker_publisher.publish(models::WorkerHealthTelemetry{
                timestamp,
                worker.name,
                worker.priority,
                worker.running,
                worker.failed,
                worker.uptime_seconds,
            });
        }

        system_publisher.publish(models::SystemHealthTelemetry{
            timestamp,
            std::chrono::duration<double>(Clock::now() - started).count(),
            cpu.sample(),
            process_memory_mb(),
            system_memory_percent(),
            running_workers,
            failed_workers,
        });

        const auto storage = platform::storage_metrics_snapshot();
        storage_publisher.publish(models::StorageHealthTelemetry{
            timestamp,
            disk_free_gb(config.data_root),
            storage.raw_bytes_written,
            storage.raw_messages_written,
            storage.pcd_points_written,
            storage.pcd_frames_written,
            storage.write_errors,
        });
        power_publisher.publish(power_thermal_sample(timestamp));
        ++report.sample_count;

        next_sample += config.sample_interval;
        interruptible_wait(next_sample, stop);
        if (Clock::now() > next_sample + config.sample_interval) {
            next_sample = Clock::now();
        }
    }
    return report;
}

}  // namespace

platform::WorkerHandle spawn_system_monitor(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    SystemMonitorConfig config,
    SystemMonitorCompletion on_complete) {
    if (config.sample_interval.count() <= 0 || config.data_root.empty()) {
        throw std::invalid_argument("invalid system-monitor configuration");
    }
    auto system_publisher = bus.publisher<models::SystemHealthTelemetry>(
        models::topics::system_health);
    auto worker_publisher = bus.publisher<models::WorkerHealthTelemetry>(
        models::topics::worker_health);
    auto storage_publisher = bus.publisher<models::StorageHealthTelemetry>(
        models::topics::storage_health);
    auto power_publisher = bus.publisher<models::PowerThermalTelemetry>(
        models::topics::power_thermal);

    return platform::spawn_worker(
        std::move(thread_config),
        [stop,
         config = std::move(config),
         system_publisher = std::move(system_publisher),
         worker_publisher = std::move(worker_publisher),
         storage_publisher = std::move(storage_publisher),
         power_publisher = std::move(power_publisher),
         on_complete = std::move(on_complete)]() mutable {
            try {
                on_complete(
                    run_system_monitor(
                        std::move(system_publisher),
                        std::move(worker_publisher),
                        std::move(storage_publisher),
                        std::move(power_publisher),
                        config,
                        stop),
                    {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
