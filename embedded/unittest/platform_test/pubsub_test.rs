//! Unit tests for the platform-level bounded in-process Pub/Sub topic.

use std::sync::Arc;

use super::*;

/// Confirms that every subscriber receives the same shared allocation.
#[test]
fn broadcasts_one_shared_message_to_all_subscribers() {
    let topic = Topic::new("test/topic", 2).unwrap();
    let first = topic.subscribe("first").unwrap();
    let second = topic.subscribe("second").unwrap();
    let publisher = topic.publisher();

    assert_eq!(publisher.publish(42_u32).unwrap(), 2);
    let first_message = first.recv().unwrap();
    let second_message = second.recv().unwrap();

    assert_eq!(*first_message, 42);
    assert!(Arc::ptr_eq(&first_message, &second_message));
}

/// Confirms that subscribers wake when the final publisher is dropped.
#[test]
fn closes_subscribers_after_last_publisher_exits() {
    let topic = Topic::<u32>::new("test/topic", 1).unwrap();
    let subscriber = topic.subscribe("reader").unwrap();
    let publisher = topic.publisher();

    drop(publisher);
    assert!(subscriber.recv().is_err());
}

/// Confirms that an unbounded-by-accident zero capacity is rejected.
#[test]
fn rejects_zero_capacity() {
    assert!(Topic::<u32>::new("test/topic", 0).is_err());
}
