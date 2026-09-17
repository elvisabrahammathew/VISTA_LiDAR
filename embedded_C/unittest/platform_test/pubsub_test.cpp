#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

#include "1_Platform/pubsub/pubsub.hpp"
#include "unittest/test.hpp"

VISTA_TEST(pubsub_broadcasts_one_shared_message) {
    vista::platform::Topic<int> topic("test/topic", 2);
    auto first = topic.subscribe("first");
    auto second = topic.subscribe("second");
    auto publisher = topic.publisher();

    VISTA_CHECK(publisher.publish(42) == 2);
    std::shared_ptr<const int> first_message;
    std::shared_ptr<const int> second_message;
    VISTA_CHECK(first.receive(first_message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(second.receive(second_message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*first_message == 42);
    VISTA_CHECK(first_message == second_message);
}

VISTA_TEST(pubsub_closes_after_last_publisher) {
    vista::platform::Topic<int> topic("test/topic", 1);
    auto subscriber = topic.subscribe("reader");
    {
        auto publisher = topic.publisher();
    }
    std::shared_ptr<const int> message;
    VISTA_CHECK(subscriber.receive(message) == vista::platform::ReceiveStatus::closed);
}

VISTA_TEST(pubsub_rejects_zero_capacity) {
    VISTA_CHECK_THROWS(vista::platform::Topic<int>("bad", 0));
}

VISTA_TEST(pubsub_new_subscriber_starts_at_next_published_message) {
    vista::platform::Topic<int> topic("test/latest", 4);
    auto publisher = topic.publisher();
    VISTA_CHECK(publisher.publish(1) == 0);

    auto subscriber = topic.subscribe("late-reader");
    VISTA_CHECK(publisher.publish(2) == 1);

    std::shared_ptr<const int> message;
    VISTA_CHECK(
        subscriber.receive(message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 2);
}

VISTA_TEST(pubsub_ring_overwrites_oldest_and_counts_subscriber_drops) {
    vista::platform::Topic<int> topic("test/ring", 2);
    auto subscriber = topic.subscribe("slow-reader");
    auto publisher = topic.publisher();

    publisher.publish(1);
    publisher.publish(2);
    publisher.publish(3);

    std::shared_ptr<const int> message;
    VISTA_CHECK(
        subscriber.receive(message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 2);
    VISTA_CHECK(subscriber.dropped_messages() == 1);
    VISTA_CHECK(
        subscriber.receive(message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 3);
}

VISTA_TEST(pubsub_subscribers_advance_independent_cursors) {
    vista::platform::Topic<int> topic("test/independent", 3);
    auto fast = topic.subscribe("fast");
    auto slow = topic.subscribe("slow");
    auto publisher = topic.publisher();
    publisher.publish(10);
    publisher.publish(20);

    std::shared_ptr<const int> fast_message;
    std::shared_ptr<const int> slow_message;
    VISTA_CHECK(fast.receive(fast_message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(fast.receive(fast_message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*fast_message == 20);

    VISTA_CHECK(slow.receive(slow_message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*slow_message == 10);
    VISTA_CHECK(slow.dropped_messages() == 0);
}

VISTA_TEST(pubsub_wait_set_reports_any_of_multiple_topics) {
    auto wait_set = std::make_shared<vista::platform::TopicWaitSet>();
    vista::platform::Topic<int> first_topic("test/first", 2);
    vista::platform::Topic<int> second_topic("test/second", 2);
    auto first = first_topic.subscribe("worker-first", wait_set, 0);
    auto second = second_topic.subscribe("worker-second", wait_set, 1);
    auto first_publisher = first_topic.publisher();
    auto second_publisher = second_topic.publisher();

    second_publisher.publish(22);
    const auto ready = wait_set->wait_for(
        vista::platform::TopicWaitSet::all_topics,
        std::chrono::milliseconds(10));
    VISTA_CHECK((ready & vista::platform::TopicWaitSet::bit(0)) == 0);
    VISTA_CHECK((ready & vista::platform::TopicWaitSet::bit(1)) != 0);

    std::shared_ptr<const int> message;
    VISTA_CHECK(second.receive(message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 22);
    VISTA_CHECK(
        (wait_set->ready_topics() & vista::platform::TopicWaitSet::bit(1)) == 0);
}

VISTA_TEST(pubsub_sequence_wraps_with_an_incremented_epoch) {
    const vista::platform::TopicSequence before{
        7, std::numeric_limits<std::uint64_t>::max()};
    const auto after = vista::platform::detail::next_topic_sequence(before);
    VISTA_CHECK(after.epoch == 8);
    VISTA_CHECK(after.value == 0);

    const vista::platform::TopicSequence cursor{
        7, std::numeric_limits<std::uint64_t>::max() - 2};
    const vista::platform::TopicSequence next{8, 3};
    VISTA_CHECK(
        vista::platform::detail::sequence_distance_saturated(cursor, next) == 6);
}

VISTA_TEST(pubsub_wait_set_rejects_more_than_64_topic_bits) {
    VISTA_CHECK_THROWS(vista::platform::TopicWaitSet::bit(64));
}
