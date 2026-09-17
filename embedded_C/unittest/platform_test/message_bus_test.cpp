#include <chrono>
#include <memory>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "unittest/test.hpp"

VISTA_TEST(message_bus_connects_named_typed_endpoints) {
    vista::platform::MessageBus bus;
    bus.configure_topic("test/numbers", 2);

    auto subscriber = bus.subscribe<int>("test/numbers", "reader");
    auto publisher = bus.publisher<int>("test/numbers");

    VISTA_CHECK(publisher.publish(42) == 1);
    std::shared_ptr<const int> message;
    VISTA_CHECK(
        subscriber.receive(message) == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 42);
}

VISTA_TEST(message_bus_rejects_a_different_type_for_the_same_name) {
    vista::platform::MessageBus bus;
    auto publisher = bus.publisher<int>("test/typed");
    VISTA_CHECK_THROWS(bus.publisher<std::string>("test/typed"));
}

VISTA_TEST(message_bus_late_subscriber_observes_closed_topic) {
    vista::platform::MessageBus bus;
    {
        auto publisher = bus.publisher<int>("test/closed");
        VISTA_CHECK(publisher.publish(1) == 0);
    }

    auto subscriber = bus.subscribe<int>("test/closed", "late-reader");
    std::shared_ptr<const int> message;
    VISTA_CHECK(
        subscriber.receive(message) == vista::platform::ReceiveStatus::closed);
}

VISTA_TEST(message_bus_worker_inputs_share_one_wait_set_and_assign_bits) {
    vista::platform::MessageBus bus;
    vista::platform::WorkerTopicInputs inputs("multi-input-worker");
    auto lidar = inputs.subscribe<int>(bus, "test/lidar");
    auto radar = inputs.subscribe<int>(bus, "test/radar");
    auto lidar_publisher = bus.publisher<int>("test/lidar");
    auto radar_publisher = bus.publisher<int>("test/radar");

    VISTA_CHECK(inputs.topic_count() == 2);
    VISTA_CHECK(lidar.ready_bit() == vista::platform::TopicWaitSet::bit(0));
    VISTA_CHECK(radar.ready_bit() == vista::platform::TopicWaitSet::bit(1));

    radar_publisher.publish(9);
    const auto ready = inputs.wait_for(std::chrono::milliseconds(10));
    VISTA_CHECK(!lidar.is_ready(ready));
    VISTA_CHECK(radar.is_ready(ready));

    std::shared_ptr<const int> message;
    VISTA_CHECK(radar.try_receive(message) ==
                vista::platform::ReceiveStatus::message);
    VISTA_CHECK(*message == 9);
    VISTA_CHECK(inputs.ready_topics() == 0);
}

VISTA_TEST(message_bus_worker_inputs_reject_wait_without_subscriptions) {
    vista::platform::WorkerTopicInputs inputs("empty-worker");
    VISTA_CHECK_THROWS(inputs.wait_for(std::chrono::milliseconds(1)));
}
