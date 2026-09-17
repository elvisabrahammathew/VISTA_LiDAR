//! Unit tests for the platform-level bounded Pub/Sub ring buffer.

use std::{sync::Arc, time::Duration};

use super::*;

fn message<T>(status: ReceiveStatus<T>) -> Arc<T> {
    match status {
        ReceiveStatus::Message(value) => value,
        ReceiveStatus::Timeout => panic!("expected a message, received timeout"),
        ReceiveStatus::Closed => panic!("expected a message, topic was closed"),
    }
}

/// Every subscriber receives the same immutable shared allocation.
#[test]
fn broadcasts_one_shared_message_to_all_subscribers() {
    let topic = Topic::new("test/topic", 2).unwrap();
    let first = topic.subscribe("first").unwrap();
    let second = topic.subscribe("second").unwrap();
    let publisher = topic.publisher().unwrap();

    assert_eq!(publisher.publish(42_u32).unwrap(), 2);
    let first_message = message(first.receive().unwrap());
    let second_message = message(second.receive().unwrap());

    assert_eq!(*first_message, 42);
    assert!(Arc::ptr_eq(&first_message, &second_message));
}

/// New subscribers intentionally start with the next publication.
#[test]
fn new_subscriber_skips_retained_history() {
    let topic = Topic::new("test/topic", 2).unwrap();
    let publisher = topic.publisher().unwrap();
    publisher.publish(1_u32).unwrap();
    let subscriber = topic.subscribe("late-reader").unwrap();
    assert!(matches!(
        subscriber.try_receive().unwrap(),
        ReceiveStatus::Timeout
    ));
    publisher.publish(2_u32).unwrap();
    assert_eq!(*message(subscriber.receive().unwrap()), 2);
}

/// A slow subscriber resumes from the oldest retained message and counts loss.
#[test]
fn overwrites_oldest_data_and_counts_drops() {
    let topic = Topic::new("test/topic", 2).unwrap();
    let subscriber = topic.subscribe("slow-reader").unwrap();
    let publisher = topic.publisher().unwrap();
    publisher.publish(1_u32).unwrap();
    publisher.publish(2_u32).unwrap();
    publisher.publish(3_u32).unwrap();

    assert_eq!(*message(subscriber.receive().unwrap()), 2);
    assert_eq!(*message(subscriber.receive().unwrap()), 3);
    assert_eq!(subscriber.dropped_messages(), 1);
}

/// Subscriber cursors advance independently through the same ring.
#[test]
fn maintains_independent_subscriber_cursors() {
    let topic = Topic::new("test/topic", 3).unwrap();
    let fast = topic.subscribe("fast").unwrap();
    let slow = topic.subscribe("slow").unwrap();
    let publisher = topic.publisher().unwrap();
    publisher.publish(10_u32).unwrap();
    publisher.publish(20_u32).unwrap();

    assert_eq!(*message(fast.receive().unwrap()), 10);
    assert_eq!(*message(fast.receive().unwrap()), 20);
    assert_eq!(*message(slow.receive().unwrap()), 10);
    assert_eq!(*message(slow.receive().unwrap()), 20);
}

/// One worker wait set can be raised by any of its subscribed topics.
#[test]
fn wait_set_supports_multiple_topics() {
    let first_topic = Topic::<u32>::new("first", 2).unwrap();
    let second_topic = Topic::<u32>::new("second", 2).unwrap();
    let wait_set = Arc::new(TopicWaitSet::default());
    let _first = first_topic
        .subscribe_with_wait_set("worker", Arc::clone(&wait_set), 0)
        .unwrap();
    let second = second_topic
        .subscribe_with_wait_set("worker", Arc::clone(&wait_set), 1)
        .unwrap();
    let publisher = second_topic.publisher().unwrap();
    publisher.publish(7_u32).unwrap();

    assert_eq!(
        wait_set
            .wait_timeout(0b11, Duration::from_millis(50))
            .unwrap(),
        0b10
    );
    assert_eq!(*message(second.receive().unwrap()), 7);
    assert_eq!(wait_set.ready_topics(), 0);
}

/// Subscribers wake and report closure when the final publisher exits.
#[test]
fn closes_subscribers_after_last_publisher_exits() {
    let topic = Topic::<u32>::new("test/topic", 1).unwrap();
    let subscriber = topic.subscribe("reader").unwrap();
    let publisher = topic.publisher().unwrap();
    drop(publisher);
    assert!(matches!(
        subscriber.receive().unwrap(),
        ReceiveStatus::Closed
    ));
}

#[test]
fn detects_sequence_wrap_without_ordering_errors() {
    let last = TopicSequence {
        epoch: 3,
        value: u64::MAX,
    };
    assert_eq!(next_sequence(last), TopicSequence { epoch: 4, value: 0 });
}

#[test]
fn rejects_zero_capacity_and_invalid_wait_bit() {
    assert!(Topic::<u32>::new("test/topic", 0).is_err());
    assert!(TopicWaitSet::bit(64).is_err());
}
