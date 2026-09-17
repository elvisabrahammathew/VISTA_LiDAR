//! Unit tests for named typed topics and worker-owned multi-topic wait sets.

use std::{io::ErrorKind, time::Duration};

use super::*;

#[test]
fn connects_named_typed_endpoints() {
    let bus = MessageBus::new(4).unwrap();
    let subscriber = bus.subscribe::<u32>("test/value", "reader").unwrap();
    let publisher = bus.publisher::<u32>("test/value").unwrap();
    publisher.publish(42).unwrap();
    match subscriber.receive().unwrap() {
        ReceiveStatus::Message(message) => assert_eq!(*message, 42),
        _ => panic!("expected one message"),
    }
}

#[test]
fn rejects_different_type_for_same_name() {
    let bus = MessageBus::new(4).unwrap();
    let _publisher = bus.publisher::<u32>("test/value").unwrap();
    let error = match bus.publisher::<String>("test/value") {
        Ok(_) => panic!("different topic type should fail"),
        Err(error) => error,
    };
    assert_eq!(error.kind(), ErrorKind::InvalidInput);
}

#[test]
fn worker_inputs_share_wait_set_and_assign_bits() {
    let bus = MessageBus::new(4).unwrap();
    let mut inputs = WorkerTopicInputs::new("worker").unwrap();
    let first = inputs.subscribe::<u32>(&bus, "test/first").unwrap();
    let second = inputs.subscribe::<u32>(&bus, "test/second").unwrap();
    assert_eq!(inputs.topic_count(), 2);
    assert_ne!(first.ready_bit(), second.ready_bit());

    let publisher = bus.publisher::<u32>("test/second").unwrap();
    publisher.publish(7).unwrap();
    let ready = inputs.wait_timeout(Duration::from_millis(50)).unwrap();
    assert!(!first.is_ready(ready));
    assert!(second.is_ready(ready));
}

#[test]
fn rejects_wait_without_subscriptions() {
    let inputs = WorkerTopicInputs::new("worker").unwrap();
    assert!(inputs.wait_timeout(Duration::from_millis(1)).is_err());
}
