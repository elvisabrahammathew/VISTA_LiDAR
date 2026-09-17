#pragma once

#include <atomic>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace vista::platform {

constexpr std::uint8_t highest_priority = 1;
constexpr std::uint8_t lowest_priority = 5;

/// Cooperative stop state shared by main and every worker.
class StopToken {
public:
    StopToken() : requested_(std::make_shared<std::atomic_bool>(false)) {}

    void request_stop() const noexcept {
        requested_->store(true, std::memory_order_release);
    }

    bool is_stop_requested() const noexcept {
        return requested_->load(std::memory_order_acquire);
    }

private:
    std::shared_ptr<std::atomic_bool> requested_;
};

/// Validated project-level priority where 1 is highest and 5 is lowest.
class ThreadPriority {
public:
    ThreadPriority() noexcept : level_(3) {}
    explicit ThreadPriority(std::uint8_t level);
    std::uint8_t level() const noexcept { return level_; }

private:
    std::uint8_t level_;
};

struct ThreadConfig {
    std::string name;
    ThreadPriority priority;

    ThreadConfig(std::string worker_name, std::uint8_t priority_level)
        : name(std::move(worker_name)), priority(priority_level) {}
};

struct NativeThreadPriority {
    std::string policy;
    int value{};
};

struct PriorityStatus {
    ThreadPriority requested{};
    bool applied{};
    NativeThreadPriority native;
    std::string error;
};

namespace detail {

PriorityStatus configure_current_thread(
    const std::string& name,
    ThreadPriority requested) noexcept;

}  // namespace detail

/// Owns one running OS worker and its startup-priority result.
class WorkerHandle {
public:
    WorkerHandle() = default;
    WorkerHandle(
        std::string name,
        PriorityStatus status,
        std::thread thread,
        std::shared_ptr<std::atomic_bool> finished,
        std::shared_ptr<std::exception_ptr> exception);

    WorkerHandle(const WorkerHandle&) = delete;
    WorkerHandle& operator=(const WorkerHandle&) = delete;
    WorkerHandle(WorkerHandle&&) noexcept = default;
    WorkerHandle& operator=(WorkerHandle&&) noexcept = default;
    ~WorkerHandle();

    const std::string& name() const noexcept { return name_; }
    const PriorityStatus& priority_status() const noexcept { return priority_status_; }
    bool is_finished() const noexcept;
    void join();

private:
    std::string name_;
    PriorityStatus priority_status_{ThreadPriority(highest_priority), false, {}, {}};
    std::thread thread_;
    std::shared_ptr<std::atomic_bool> finished_;
    std::shared_ptr<std::exception_ptr> exception_;
};

/// Starts one named worker and waits until its native priority has been attempted.
template <typename Worker>
WorkerHandle spawn_worker(ThreadConfig config, Worker worker) {
    auto finished = std::make_shared<std::atomic_bool>(false);
    auto exception = std::make_shared<std::exception_ptr>();
    std::promise<PriorityStatus> startup_promise;
    auto startup_future = startup_promise.get_future();
    const auto name = config.name;
    const auto requested = config.priority;

    std::thread thread(
        [name,
         requested,
         worker = std::move(worker),
         finished,
         exception,
         startup = std::move(startup_promise)]() mutable {
            startup.set_value(detail::configure_current_thread(name, requested));
            try {
                worker();
            } catch (...) {
                *exception = std::current_exception();
            }
            finished->store(true, std::memory_order_release);
        });

    auto status = startup_future.get();
    return WorkerHandle(
        std::move(config.name),
        std::move(status),
        std::move(thread),
        std::move(finished),
        std::move(exception));
}

const char* platform_name() noexcept;

}  // namespace vista::platform
