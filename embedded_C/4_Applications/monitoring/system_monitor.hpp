#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"

namespace vista::application {

struct SystemMonitorConfig {
    std::chrono::milliseconds sample_interval{2'000};
    std::filesystem::path data_root{"../data"};
};

struct SystemMonitorReport {
    std::uint64_t sample_count{};
};

using SystemMonitorCompletion =
    std::function<void(std::optional<SystemMonitorReport>, std::string)>;

platform::WorkerHandle spawn_system_monitor(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    SystemMonitorConfig config,
    SystemMonitorCompletion on_complete);

}  // namespace vista::application
