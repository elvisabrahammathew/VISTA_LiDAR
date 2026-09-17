#include "1_Platform/pubsub/pubsub.hpp"

#include <limits>
#include <stdexcept>

namespace vista::platform {

TopicWaitSet::Mask TopicWaitSet::bit(std::uint8_t bit_index) {
    if (bit_index >= maximum_topics) {
        throw std::out_of_range("topic wait-set bit must be between 0 and 63");
    }
    return Mask{1} << bit_index;
}

TopicWaitSet::Mask TopicWaitSet::wait(Mask interested_topics) {
    if (interested_topics == 0) {
        throw std::invalid_argument("wait-set topic mask cannot be zero");
    }
    std::unique_lock lock(mutex_);
    condition_.wait(lock, [this, interested_topics] {
        return (ready_topics_ & interested_topics) != 0;
    });
    return ready_topics_ & interested_topics;
}

TopicWaitSet::Mask TopicWaitSet::ready_topics() const noexcept {
    std::lock_guard lock(mutex_);
    return ready_topics_;
}

void TopicWaitSet::set_ready(Mask topics) noexcept {
    {
        std::lock_guard lock(mutex_);
        ready_topics_ |= topics;
    }
    condition_.notify_all();
}

void TopicWaitSet::clear_ready(Mask topics) noexcept {
    std::lock_guard lock(mutex_);
    ready_topics_ &= ~topics;
}

namespace detail {

void validate_topic_capacity(std::size_t capacity) {
    if (capacity == 0) {
        throw std::invalid_argument("topic capacity must be greater than zero");
    }
}

TopicSequence next_topic_sequence(TopicSequence sequence) noexcept {
    if (sequence.value == std::numeric_limits<std::uint64_t>::max()) {
        sequence.value = 0;
        ++sequence.epoch;
    } else {
        ++sequence.value;
    }
    return sequence;
}

std::uint64_t sequence_distance_saturated(
    TopicSequence first,
    TopicSequence last) noexcept {
    const auto epoch_distance = last.epoch - first.epoch;
    if (epoch_distance == 0) {
        return last.value - first.value;
    }
    if (epoch_distance != 1 || first.value == 0) {
        return std::numeric_limits<std::uint64_t>::max();
    }

    const auto distance_to_next_epoch =
        std::numeric_limits<std::uint64_t>::max() - first.value + 1;
    if (last.value >
        std::numeric_limits<std::uint64_t>::max() - distance_to_next_epoch) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return distance_to_next_epoch + last.value;
}

std::uint64_t saturating_add(
    std::uint64_t left,
    std::uint64_t right) noexcept {
    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    return right > maximum - left ? maximum : left + right;
}

}  // namespace detail

}  // namespace vista::platform
