#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "1_Platform/pubsub/pubsub.hpp"

namespace vista::platform {

/// Owns named, typed topics shared by otherwise independent workers.
class MessageBus {
private:
    class TopicHolderBase;

public:
    explicit MessageBus(std::size_t default_capacity = 16);

    MessageBus(const MessageBus&) = delete;
    MessageBus& operator=(const MessageBus&) = delete;

    ~MessageBus();

    /// Sets the queue capacity used when this named topic is first requested.
    void configure_topic(std::string name, std::size_t capacity);

    /// Returns a publisher for a named topic, creating the topic when necessary.
    template <typename T>
    TopicPublisher<T> publisher(const std::string& topic_name) {
        return topic<T>(topic_name)->publisher();
    }

    /// Registers a cursor at the next message published on the named topic.
    template <typename T>
    TopicSubscriber<T> subscribe(
        const std::string& topic_name,
        std::string subscriber_name) {
        return topic<T>(topic_name)->subscribe(std::move(subscriber_name));
    }

    /// Registers the topic in a worker-owned wait set for multi-topic waiting.
    template <typename T>
    TopicSubscriber<T> subscribe(
        const std::string& topic_name,
        std::string subscriber_name,
        std::shared_ptr<TopicWaitSet> wait_set,
        std::uint8_t wait_bit) {
        return topic<T>(topic_name)->subscribe(
            std::move(subscriber_name), std::move(wait_set), wait_bit);
    }

    /// Wakes all subscribers and prevents creation of new topic endpoints.
    void close() noexcept;

private:
    class TopicHolderBase {
    public:
        virtual ~TopicHolderBase() = default;
        virtual std::type_index message_type() const noexcept = 0;
        virtual void close() noexcept = 0;
    };

    template <typename T>
    class TopicHolder final : public TopicHolderBase {
    public:
        TopicHolder(std::string name, std::size_t capacity)
            : topic_(std::make_shared<Topic<T>>(std::move(name), capacity)) {}

        std::type_index message_type() const noexcept override {
            return typeid(T);
        }

        void close() noexcept override {
            topic_->close();
        }

        const std::shared_ptr<Topic<T>>& topic() const noexcept {
            return topic_;
        }

    private:
        std::shared_ptr<Topic<T>> topic_;
    };

    template <typename T>
    std::shared_ptr<Topic<T>> topic(const std::string& name) {
        std::lock_guard lock(mutex_);
        if (closed_) {
            throw std::runtime_error("cannot use closed message bus topic '" + name + "'");
        }

        const auto existing = topics_.find(name);
        if (existing != topics_.end()) {
            if (existing->second->message_type() != std::type_index(typeid(T))) {
                throw std::runtime_error(
                    "topic '" + name + "' was requested with a different message type");
            }
            return static_cast<TopicHolder<T>&>(*existing->second).topic();
        }

        auto capacity = default_capacity_;
        const auto configured = capacities_.find(name);
        if (configured != capacities_.end()) {
            capacity = configured->second;
        }
        auto holder = std::make_shared<TopicHolder<T>>(name, capacity);
        auto typed_topic = holder->topic();
        topics_.emplace(name, std::move(holder));
        return typed_topic;
    }

    std::size_t default_capacity_;
    std::mutex mutex_;
    std::unordered_map<std::string, std::size_t> capacities_;
    std::unordered_map<std::string, std::shared_ptr<TopicHolderBase>> topics_;
    bool closed_{false};
};

/// One typed input registered in a worker-owned multi-topic WaitSet.
template <typename T>
class WorkerTopicInput {
public:
    using Message = std::shared_ptr<const T>;

    WorkerTopicInput(const WorkerTopicInput&) = delete;
    WorkerTopicInput& operator=(const WorkerTopicInput&) = delete;
    WorkerTopicInput(WorkerTopicInput&&) noexcept = default;
    WorkerTopicInput& operator=(WorkerTopicInput&&) noexcept = default;

    bool is_ready(TopicWaitSet::Mask ready_topics) const noexcept {
        return (ready_topics & ready_bit_) != 0;
    }

    TopicWaitSet::Mask ready_bit() const noexcept {
        return ready_bit_;
    }

    ReceiveStatus try_receive(Message& message) {
        return subscriber_.try_receive(message);
    }

    ReceiveStatus receive(Message& message) {
        return subscriber_.receive(message);
    }

    std::uint64_t dropped_messages() const noexcept {
        return subscriber_.dropped_messages();
    }

    std::size_t pending_messages() const noexcept {
        return subscriber_.pending_messages();
    }

    std::size_t capacity() const noexcept {
        return subscriber_.capacity();
    }

    TopicSubscriber<T> take_subscriber() && {
        return std::move(subscriber_);
    }

private:
    friend class WorkerTopicInputs;

    WorkerTopicInput(
        TopicSubscriber<T> subscriber,
        TopicWaitSet::Mask ready_bit)
        : subscriber_(std::move(subscriber)), ready_bit_(ready_bit) {}

    TopicSubscriber<T> subscriber_;
    TopicWaitSet::Mask ready_bit_{};
};

/// Owns one 64-bit WaitSet and automatically assigns one input bit per worker topic.
class WorkerTopicInputs {
public:
    explicit WorkerTopicInputs(std::string worker_name)
        : worker_name_(std::move(worker_name)),
          wait_set_(std::make_shared<TopicWaitSet>()) {
        if (worker_name_.empty()) {
            throw std::invalid_argument("worker topic-input name cannot be empty");
        }
    }

    WorkerTopicInputs(const WorkerTopicInputs&) = delete;
    WorkerTopicInputs& operator=(const WorkerTopicInputs&) = delete;
    WorkerTopicInputs(WorkerTopicInputs&&) noexcept = default;
    WorkerTopicInputs& operator=(WorkerTopicInputs&&) noexcept = default;

    template <typename T>
    WorkerTopicInput<T> subscribe(
        MessageBus& bus,
        const std::string& topic_name) {
        if (next_bit_ >= TopicWaitSet::maximum_topics) {
            throw std::runtime_error(
                "worker cannot subscribe to more than 64 topics");
        }

        const auto bit_index = static_cast<std::uint8_t>(next_bit_);
        const auto ready_bit = TopicWaitSet::bit(bit_index);
        auto subscriber = bus.subscribe<T>(
            topic_name, worker_name_, wait_set_, bit_index);
        ++next_bit_;
        subscribed_topics_ |= ready_bit;
        return WorkerTopicInput<T>(std::move(subscriber), ready_bit);
    }

    TopicWaitSet::Mask wait() {
        ensure_has_inputs();
        return wait_set_->wait(subscribed_topics_);
    }

    template <typename Rep, typename Period>
    TopicWaitSet::Mask wait_for(
        const std::chrono::duration<Rep, Period>& timeout) {
        ensure_has_inputs();
        return wait_set_->wait_for(subscribed_topics_, timeout);
    }

    TopicWaitSet::Mask ready_topics() const noexcept {
        return wait_set_->ready_topics() & subscribed_topics_;
    }

    std::size_t topic_count() const noexcept {
        return next_bit_;
    }

private:
    void ensure_has_inputs() const {
        if (subscribed_topics_ == 0) {
            throw std::logic_error("worker has no subscribed input topics");
        }
    }

    std::string worker_name_;
    std::shared_ptr<TopicWaitSet> wait_set_;
    TopicWaitSet::Mask subscribed_topics_{};
    std::size_t next_bit_{};
};

}  // namespace vista::platform
