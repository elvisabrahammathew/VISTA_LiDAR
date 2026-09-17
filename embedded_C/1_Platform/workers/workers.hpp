#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"

namespace vista {
struct AppConfig;
struct RuntimeConfig;
}  // namespace vista

namespace vista::platform {

/// Stores the report or error returned asynchronously by one worker.
template <typename Report>
struct WorkerResult {
    std::optional<Report> report;
    std::string error;
};

/// Creates the completion callback passed to a spawn_* function.
template <typename Report>
auto completion_for(const std::shared_ptr<WorkerResult<Report>>& result) {
    return [result](std::optional<Report> report, std::string error) {
        result->report = std::move(report);
        result->error = std::move(error);
    };
}

/// Validates dependencies between the worker enable flags selected in main.
void validate_worker_selection(
    const AppConfig& app,
    const RuntimeConfig& runtime);

/// Prints native-priority status and stores a newly-created worker handle.
void add_worker(
    std::vector<WorkerHandle>& workers,
    WorkerHandle handle);

/// Blocks until Ctrl+C/SIGTERM or a worker requests cooperative shutdown.
void wait_for_shutdown_request(const StopToken& stop);

/// Requests shutdown, closes all topics, and joins without hiding a startup error.
void stop_and_join_noexcept(
    std::vector<WorkerHandle>& workers,
    const StopToken& stop,
    MessageBus& bus) noexcept;

/// Joins every worker and appends any uncaught worker failures.
void join_workers(
    std::vector<WorkerHandle>& workers,
    const StopToken& stop,
    MessageBus& bus,
    std::vector<std::string>& failures);

/// Appends the completion error for one worker that was actually started.
template <typename Report>
void collect_worker_error(
    std::vector<std::string>& failures,
    const std::string& name,
    const std::shared_ptr<WorkerResult<Report>>& result,
    bool was_started) {
    if (!was_started) {
        return;
    }
    if (!result->error.empty()) {
        failures.push_back(name + ": " + result->error);
    } else if (!result->report) {
        failures.push_back(name + ": worker exited without a completion report");
    }
}

/// Combines collected failures and throws one application-facing exception.
void throw_if_worker_failures(const std::vector<std::string>& failures);

}  // namespace vista::platform
