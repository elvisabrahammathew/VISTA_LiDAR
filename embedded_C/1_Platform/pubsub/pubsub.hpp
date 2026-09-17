#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vista::platform {

enum class ReceiveStatus {
    message,
    timeout,
    closed,
};

/// Identifies one logical message even when the 64-bit value wraps to zero.
struct TopicSequence {
    std::uint64_t epoch{};
    std::uint64_t value{};
};

inline bool operator==(
    const TopicSequence& left,
    const TopicSequence& right) noexcept {
    return left.epoch == right.epoch && left.value == right.value;
}

inline bool operator!=(
    const TopicSequence& left,
    const TopicSequence& right) noexcept {
    return !(left == right);
}

namespace detail {

// Implemented in pubsub.cpp because these helpers do not depend on message type T.
void validate_topic_capacity(std::size_t capacity);
TopicSequence next_topic_sequence(TopicSequence sequence) noexcept;
std::uint64_t sequence_distance_saturated(
    TopicSequence first,
    TopicSequence last) noexcept;
std::uint64_t saturating_add(
    std::uint64_t left,
    std::uint64_t right) noexcept;

}  // namespace detail

/// Event-flags object used by one worker to wait for up to 64 subscribed topics.
class TopicWaitSet {
public:
    using Mask = std::uint64_t;
    static constexpr std::size_t maximum_topics = 64;
    static constexpr Mask all_topics = std::numeric_limits<Mask>::max();

    static Mask bit(std::uint8_t bit_index);

    /// Blocks without polling until at least one interested topic is ready.
    Mask wait(Mask interested_topics = all_topics);

    /// Returns zero on timeout; readiness bits are cleared by subscribers.
    template <typename Rep, typename Period>
    Mask wait_for(
        Mask interested_topics,
        const std::chrono::duration<Rep, Period>& timeout) {
        if (interested_topics == 0) {
            throw std::invalid_argument("wait-set topic mask cannot be zero");
        }
        std::unique_lock lock(mutex_);
        if (!condition_.wait_for(lock, timeout, [this, interested_topics] {
                return (ready_topics_ & interested_topics) != 0;
            })) {
            return 0;
        }
        return ready_topics_ & interested_topics;
    }

    Mask ready_topics() const noexcept;

    // Topic/subscriber infrastructure uses these to maintain level-triggered bits.
    void set_ready(Mask topics) noexcept;
    void clear_ready(Mask topics) noexcept;

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    Mask ready_topics_{};
};

template <typename T>
class TopicPublisher;

template <typename T>
class TopicSubscriber;

namespace detail {

template <typename T>
struct RingSlot {
    TopicSequence sequence;
    std::shared_ptr<const T> message;
    bool occupied{false};
};

template <typename T>
struct SubscriberState {
    TopicSequence cursor;
    std::size_t read_index{};
    std::uint64_t dropped_messages{};
    std::shared_ptr<TopicWaitSet> wait_set;
    TopicWaitSet::Mask ready_bit{};
    bool active{true};
};

template <typename T>
struct TopicState {
    explicit TopicState(std::size_t ring_capacity)
        : capacity(ring_capacity), ring(ring_capacity) {}

    std::size_t oldest_index() const noexcept {
        return retained_count < capacity ? 0 : write_index;
    }

    std::size_t capacity;
    std::vector<RingSlot<T>> ring;
    std::size_t write_index{};
    std::size_t retained_count{};
    TopicSequence next_write_sequence{};
    std::mutex mutex;
    std::vector<std::weak_ptr<SubscriberState<T>>> subscribers;
    std::size_t publisher_count{};
    bool closed{false};
};

/// Closes a topic once and wakes every subscriber waiting for data.
template <typename T>
void close_topic(const std::shared_ptr<TopicState<T>>& state) noexcept {
    std::lock_guard lock(state->mutex);
    if (state->closed) {
        return;
    }
    state->closed = true;
    for (auto iterator = state->subscribers.begin();
         iterator != state->subscribers.end();) {
        if (auto subscriber = iterator->lock()) {
            if (subscriber->active) {
                subscriber->wait_set->set_ready(subscriber->ready_bit);
            }
            ++iterator;
        } else {
            iterator = state->subscribers.erase(iterator);
        }
    }
}

}  // namespace detail

/// One bounded broadcast ring whose immutable messages are shared by subscribers.
template <typename T>
class Topic {
public:
    Topic(std::string name, std::size_t capacity)
        : name_(std::move(name)) {
        detail::validate_topic_capacity(capacity);
        state_ = std::make_shared<detail::TopicState<T>>(capacity);
    }

    TopicPublisher<T> publisher() const {
        return TopicPublisher<T>(name_, state_);
    }

    /// Starts at the next message published; retained historical data is skipped.
    TopicSubscriber<T> subscribe(std::string subscriber_name) const {
        return subscribe(
            std::move(subscriber_name),
            std::make_shared<TopicWaitSet>(),
            0);
    }

    /// Registers this topic in a worker-owned 64-bit wait set.
    TopicSubscriber<T> subscribe(
        std::string subscriber_name,
        std::shared_ptr<TopicWaitSet> wait_set,
        std::uint8_t wait_bit) const {
        if (!wait_set) {
            throw std::invalid_argument("subscriber wait set cannot be null");
        }
        const auto ready_bit = TopicWaitSet::bit(wait_bit);
        auto subscriber = std::make_shared<detail::SubscriberState<T>>();
        subscriber->wait_set = std::move(wait_set);
        subscriber->ready_bit = ready_bit;

        {
            std::lock_guard lock(state_->mutex);
            // Registration and cursor initialization are atomic with publication.
            subscriber->cursor = state_->next_write_sequence;
            subscriber->read_index = state_->write_index;
            if (!state_->closed) {
                state_->subscribers.emplace_back(subscriber);
            } else {
                subscriber->wait_set->set_ready(ready_bit);
            }
        }

        return TopicSubscriber<T>(
            name_, std::move(subscriber_name), state_, std::move(subscriber));
    }

    void close() const noexcept {
        detail::close_topic(state_);
    }

private:
    std::string name_;
    std::shared_ptr<detail::TopicState<T>> state_;
};

/// Publishes once into the shared ring and signals every active subscriber.
template <typename T>
class TopicPublisher {
public:
    using Message = std::shared_ptr<const T>;

    TopicPublisher(const TopicPublisher& other)
        : name_(other.name_), state_(other.state_) {
        retain();
    }

    TopicPublisher(TopicPublisher&& other) noexcept
        : name_(std::move(other.name_)), state_(std::move(other.state_)) {}

    TopicPublisher& operator=(const TopicPublisher& other) {
        if (this != &other) {
            release();
            name_ = other.name_;
            state_ = other.state_;
            retain();
        }
        return *this;
    }

    TopicPublisher& operator=(TopicPublisher&& other) noexcept {
        if (this != &other) {
            release();
            name_ = std::move(other.name_);
            state_ = std::move(other.state_);
        }
        return *this;
    }

    ~TopicPublisher() {
        release();
    }

    std::size_t publish(T message) const {
        return publish_shared(std::make_shared<const T>(std::move(message)));
    }

    std::size_t publish_shared(Message message) const {
        if (!state_) {
            throw std::runtime_error("publisher has no topic state");
        }
        if (!message) {
            throw std::invalid_argument("published message cannot be null");
        }

        std::lock_guard lock(state_->mutex);
        if (state_->closed) {
            return 0;
        }

        auto& slot = state_->ring[state_->write_index];
        slot.sequence = state_->next_write_sequence;
        slot.message = std::move(message);
        slot.occupied = true;

        state_->write_index = (state_->write_index + 1) % state_->capacity;
        if (state_->retained_count < state_->capacity) {
            ++state_->retained_count;
        }
        state_->next_write_sequence =
            detail::next_topic_sequence(state_->next_write_sequence);

        std::size_t signalled = 0;
        for (auto iterator = state_->subscribers.begin();
             iterator != state_->subscribers.end();) {
            if (auto subscriber = iterator->lock()) {
                if (subscriber->active) {
                    subscriber->wait_set->set_ready(subscriber->ready_bit);
                    ++signalled;
                }
                ++iterator;
            } else {
                iterator = state_->subscribers.erase(iterator);
            }
        }
        return signalled;
    }

    const std::string& name() const noexcept {
        return name_;
    }

private:
    friend class Topic<T>;

    TopicPublisher(std::string name, std::shared_ptr<detail::TopicState<T>> state)
        : name_(std::move(name)), state_(std::move(state)) {
        retain();
    }

    void retain() {
        if (!state_) {
            return;
        }
        std::lock_guard lock(state_->mutex);
        if (state_->closed) {
            throw std::runtime_error(
                "cannot create a publisher for closed topic '" + name_ + "'");
        }
        ++state_->publisher_count;
    }

    void release() noexcept {
        if (!state_) {
            return;
        }
        {
            std::lock_guard lock(state_->mutex);
            if (state_->publisher_count > 0) {
                --state_->publisher_count;
                if (state_->publisher_count == 0 && !state_->closed) {
                    state_->closed = true;
                    for (auto iterator = state_->subscribers.begin();
                         iterator != state_->subscribers.end();) {
                        if (auto subscriber = iterator->lock()) {
                            if (subscriber->active) {
                                subscriber->wait_set->set_ready(
                                    subscriber->ready_bit);
                            }
                            ++iterator;
                        } else {
                            iterator = state_->subscribers.erase(iterator);
                        }
                    }
                }
            }
        }
        state_.reset();
    }

    std::string name_;
    std::shared_ptr<detail::TopicState<T>> state_;
};

/// Reads the shared ring through an independent cursor and readiness flag.
template <typename T>
class TopicSubscriber {
public:
    using Message = std::shared_ptr<const T>;

    TopicSubscriber(const TopicSubscriber&) = delete;
    TopicSubscriber& operator=(const TopicSubscriber&) = delete;

    TopicSubscriber(TopicSubscriber&& other) noexcept
        : topic_name_(std::move(other.topic_name_)),
          subscriber_name_(std::move(other.subscriber_name_)),
          state_(std::move(other.state_)),
          subscriber_(std::move(other.subscriber_)) {}

    TopicSubscriber& operator=(TopicSubscriber&& other) noexcept {
        if (this != &other) {
            close();
            topic_name_ = std::move(other.topic_name_);
            subscriber_name_ = std::move(other.subscriber_name_);
            state_ = std::move(other.state_);
            subscriber_ = std::move(other.subscriber_);
        }
        return *this;
    }

    ~TopicSubscriber() {
        close();
    }

    ReceiveStatus receive(Message& message) {
        for (;;) {
            {
                std::lock_guard lock(state_->mutex);
                const auto status = try_receive_locked(message);
                if (status != ReceiveStatus::timeout) {
                    return status;
                }
            }
            subscriber_->wait_set->wait(subscriber_->ready_bit);
        }
    }

    /// Checks the ring once without blocking; useful after a multi-topic wait.
    ReceiveStatus try_receive(Message& message) {
        std::lock_guard lock(state_->mutex);
        return try_receive_locked(message);
    }

    template <typename Rep, typename Period>
    ReceiveStatus receive_for(
        Message& message,
        const std::chrono::duration<Rep, Period>& timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            {
                std::lock_guard lock(state_->mutex);
                const auto status = try_receive_locked(message);
                if (status != ReceiveStatus::timeout) {
                    return status;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline ||
                subscriber_->wait_set->wait_for(
                    subscriber_->ready_bit, deadline - now) == 0) {
                return ReceiveStatus::timeout;
            }
        }
    }

    const std::string& topic_name() const noexcept {
        return topic_name_;
    }

    const std::string& subscriber_name() const noexcept {
        return subscriber_name_;
    }

    std::uint64_t dropped_messages() const noexcept {
        if (!state_ || !subscriber_) {
            return 0;
        }
        std::lock_guard lock(state_->mutex);
        return subscriber_->dropped_messages;
    }

    TopicSequence cursor() const noexcept {
        if (!state_ || !subscriber_) {
            return {};
        }
        std::lock_guard lock(state_->mutex);
        return subscriber_->cursor;
    }

private:
    friend class Topic<T>;

    TopicSubscriber(
        std::string topic_name,
        std::string subscriber_name,
        std::shared_ptr<detail::TopicState<T>> state,
        std::shared_ptr<detail::SubscriberState<T>> subscriber)
        : topic_name_(std::move(topic_name)),
          subscriber_name_(std::move(subscriber_name)),
          state_(std::move(state)),
          subscriber_(std::move(subscriber)) {}

    ReceiveStatus try_receive_locked(Message& message) {
        if (!subscriber_->active) {
            return ReceiveStatus::closed;
        }

        if (subscriber_->cursor != state_->next_write_sequence) {
            auto& expected_slot = state_->ring[subscriber_->read_index];
            if (!expected_slot.occupied ||
                expected_slot.sequence != subscriber_->cursor) {
                // The publisher wrapped the ring before this subscriber copied
                // the message. Move its cursor to the oldest retained slot.
                const auto oldest_index = state_->oldest_index();
                const auto& oldest_slot = state_->ring[oldest_index];
                if (!oldest_slot.occupied) {
                    throw std::logic_error("ring buffer has no retained message");
                }
                const auto dropped = detail::sequence_distance_saturated(
                    subscriber_->cursor, oldest_slot.sequence);
                subscriber_->dropped_messages = detail::saturating_add(
                    subscriber_->dropped_messages, dropped);
                subscriber_->cursor = oldest_slot.sequence;
                subscriber_->read_index = oldest_index;
            }

            auto& slot = state_->ring[subscriber_->read_index];
            if (!slot.occupied || slot.sequence != subscriber_->cursor) {
                throw std::logic_error("ring-buffer subscriber cursor mismatch");
            }
            message = slot.message;
            subscriber_->cursor =
                detail::next_topic_sequence(subscriber_->cursor);
            subscriber_->read_index =
                (subscriber_->read_index + 1) % state_->capacity;

            if (subscriber_->cursor == state_->next_write_sequence) {
                subscriber_->wait_set->clear_ready(subscriber_->ready_bit);
            }
            return ReceiveStatus::message;
        }

        subscriber_->wait_set->clear_ready(subscriber_->ready_bit);
        return state_->closed ? ReceiveStatus::closed : ReceiveStatus::timeout;
    }

    void close() noexcept {
        if (!state_ || !subscriber_) {
            return;
        }
        {
            std::lock_guard lock(state_->mutex);
            subscriber_->active = false;
            subscriber_->wait_set->clear_ready(subscriber_->ready_bit);
        }
        subscriber_.reset();
        state_.reset();
    }

    std::string topic_name_;
    std::string subscriber_name_;
    std::shared_ptr<detail::TopicState<T>> state_;
    std::shared_ptr<detail::SubscriberState<T>> subscriber_;
};

}  // namespace vista::platform
