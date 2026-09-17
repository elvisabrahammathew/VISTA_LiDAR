#include "1_Platform/message_bus/message_bus.hpp"

namespace vista::platform {

MessageBus::MessageBus(std::size_t default_capacity)
    : default_capacity_(default_capacity) {
    if (default_capacity_ == 0) {
        throw std::invalid_argument(
            "message bus default capacity must be greater than zero");
    }
}

MessageBus::~MessageBus() {
    close();
}

void MessageBus::configure_topic(std::string name, std::size_t capacity) {
    if (capacity == 0) {
        throw std::invalid_argument(
            "message bus topic capacity must be greater than zero");
    }
    std::lock_guard lock(mutex_);
    if (closed_) {
        throw std::runtime_error("cannot configure a closed message bus");
    }
    if (topics_.find(name) != topics_.end()) {
        throw std::runtime_error(
            "cannot change capacity after topic '" + name + "' was created");
    }
    capacities_.insert_or_assign(std::move(name), capacity);
}

void MessageBus::close() noexcept {
    std::vector<std::shared_ptr<TopicHolderBase>> topics;
    {
        std::lock_guard lock(mutex_);
        if (closed_) {
            return;
        }
        closed_ = true;
        topics.reserve(topics_.size());
        for (const auto& entry : topics_) {
            topics.push_back(entry.second);
        }
    }
    for (const auto& topic : topics) {
        topic->close();
    }
}

}  // namespace vista::platform
