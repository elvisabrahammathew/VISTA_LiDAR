#include "1_Platform/workers/workers.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "config.hpp"

namespace vista::platform {

void validate_worker_selection(
    const AppConfig& app,
    const RuntimeConfig& runtime) {
    if (runtime.threads.lidar_decode.enabled &&
        !runtime.threads.lidar_read.enabled) {
        throw std::invalid_argument(
            "lidar-decode requires the lidar-read worker");
    }
    if (runtime.threads.preprocessing.enabled &&
        !runtime.threads.lidar_decode.enabled) {
        throw std::invalid_argument(
            "pointcloud-preprocessing requires the lidar-decode worker");
    }
    if (app.raw_path && runtime.threads.raw_logger.enabled &&
        !runtime.threads.lidar_read.enabled) {
        throw std::invalid_argument("raw-logger requires the lidar-read worker");
    }
    if (app.pcd_path && runtime.threads.pcd_logger.enabled &&
        !runtime.threads.preprocessing.enabled) {
        throw std::invalid_argument(
            "pcd-logger requires the pointcloud-preprocessing worker");
    }
    if (app.pointcloud_websocket.enabled &&
        runtime.threads.pointcloud_websocket.enabled &&
        !runtime.threads.preprocessing.enabled) {
        throw std::invalid_argument(
            "pointcloud-websocket requires the pointcloud-preprocessing worker");
    }
}

namespace {

volatile std::sig_atomic_t shutdown_signal_received = 0;

extern "C" void record_shutdown_signal(int) {
    shutdown_signal_received = 1;
}

void print_priority_status(const WorkerHandle& handle) {
    const auto& status = handle.priority_status();
    if (status.applied) {
        std::cout << "Thread '" << handle.name() << "': priority "
                  << static_cast<int>(status.requested.level()) << " -> "
                  << status.native.policy << ' ' << status.native.value << '\n';
    } else {
        std::cerr << "Warning: thread '" << handle.name()
                  << "' kept inherited priority (requested "
                  << static_cast<int>(status.requested.level()) << "): "
                  << status.error << '\n';
    }
}

}  // namespace

void add_worker(
    std::vector<WorkerHandle>& workers,
    WorkerHandle handle) {
    print_priority_status(handle);
    workers.push_back(std::move(handle));
}

void wait_for_shutdown_request(const StopToken& stop) {
    shutdown_signal_received = 0;
    const auto previous_interrupt = std::signal(SIGINT, record_shutdown_signal);
    const auto previous_terminate = std::signal(SIGTERM, record_shutdown_signal);
    if (previous_interrupt == SIG_ERR || previous_terminate == SIG_ERR) {
        throw std::runtime_error("could not install shutdown signal handlers");
    }

    while (!stop.is_stop_requested() && shutdown_signal_received == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::signal(SIGINT, previous_interrupt);
    std::signal(SIGTERM, previous_terminate);
    stop.request_stop();
}

void stop_and_join_noexcept(
    std::vector<WorkerHandle>& workers,
    const StopToken& stop,
    MessageBus& bus) noexcept {
    stop.request_stop();
    bus.close();
    for (auto& worker : workers) {
        try {
            worker.join();
        } catch (...) {
            // The caller is already preserving a more relevant startup error.
        }
    }
}

void join_workers(
    std::vector<WorkerHandle>& workers,
    const StopToken& stop,
    MessageBus& bus,
    std::vector<std::string>& failures) {
    for (auto& worker : workers) {
        try {
            worker.join();
        } catch (const std::exception& error) {
            failures.push_back(worker.name() + ": " + error.what());
            stop.request_stop();
            bus.close();
        } catch (...) {
            failures.push_back(worker.name() + ": unknown worker failure");
            stop.request_stop();
            bus.close();
        }
    }
}

void throw_if_worker_failures(const std::vector<std::string>& failures) {
    if (failures.empty()) {
        return;
    }

    std::string message;
    for (const auto& failure : failures) {
        if (!message.empty()) {
            message += "; ";
        }
        message += failure;
    }
    throw std::runtime_error(message);
}

}  // namespace vista::platform
