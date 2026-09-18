#include "1_Platform/threading/threading.hpp"

#include <chrono>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace vista::platform {

namespace {

struct WorkerRuntimeState {
    std::uint8_t priority{};
    bool running{};
    bool failed{};
    std::chrono::steady_clock::time_point started_at{};
    std::chrono::steady_clock::time_point finished_at{};
};

std::mutex runtime_mutex;
std::unordered_map<std::string, WorkerRuntimeState> runtime_workers;

}  // namespace

ThreadPriority::ThreadPriority(std::uint8_t level) : level_(level) {
    if (level < highest_priority || level > lowest_priority) {
        std::ostringstream message;
        message << "thread priority must be between "
                << static_cast<int>(highest_priority) << " (highest) and "
                << static_cast<int>(lowest_priority) << " (lowest); received "
                << static_cast<int>(level);
        throw std::invalid_argument(message.str());
    }
}

WorkerHandle::WorkerHandle(
    std::string name,
    PriorityStatus status,
    std::thread thread,
    std::shared_ptr<std::atomic_bool> finished,
    std::shared_ptr<std::exception_ptr> exception)
    : name_(std::move(name)),
      priority_status_(std::move(status)),
      thread_(std::move(thread)),
      finished_(std::move(finished)),
      exception_(std::move(exception)) {}

WorkerHandle::~WorkerHandle() {
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool WorkerHandle::is_finished() const noexcept {
    return finished_ && finished_->load(std::memory_order_acquire);
}

void WorkerHandle::join() {
    if (thread_.joinable()) {
        thread_.join();
    }
    if (exception_ && *exception_) {
        std::rethrow_exception(*exception_);
    }
}

std::vector<WorkerRuntimeStatus> worker_runtime_snapshot() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(runtime_mutex);
    std::vector<WorkerRuntimeStatus> result;
    result.reserve(runtime_workers.size());
    for (const auto& entry : runtime_workers) {
        const auto& state = entry.second;
        const auto end = state.running ? now : state.finished_at;
        result.push_back({
            entry.first,
            state.priority,
            state.running,
            state.failed,
            std::chrono::duration<double>(end - state.started_at).count(),
        });
    }
    return result;
}

namespace detail {

void record_worker_started(
    const std::string& name,
    ThreadPriority priority) noexcept {
    try {
        std::lock_guard lock(runtime_mutex);
        runtime_workers[name] = WorkerRuntimeState{
            priority.level(), true, false, std::chrono::steady_clock::now(), {}};
    } catch (...) {
        // Monitoring must never prevent a worker from starting.
    }
}

void record_worker_finished(
    const std::string& name,
    bool failed) noexcept {
    try {
        std::lock_guard lock(runtime_mutex);
        const auto existing = runtime_workers.find(name);
        if (existing != runtime_workers.end()) {
            existing->second.running = false;
            existing->second.failed = failed;
            existing->second.finished_at = std::chrono::steady_clock::now();
        }
    } catch (...) {
        // Monitoring must never change worker shutdown behavior.
    }
}

}  // namespace detail

}  // namespace vista::platform
