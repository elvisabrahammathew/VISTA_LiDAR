#include "1_Platform/threading/threading.hpp"

#include <sstream>

namespace vista::platform {

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

}  // namespace vista::platform
