//! Bounded in-process Pub/Sub ring buffers with independent subscriber cursors.

use std::{
    io,
    sync::{Arc, Condvar, Mutex, MutexGuard, Weak},
    time::{Duration, Instant},
};

/// Identifies a logical message even when the 64-bit value wraps to zero.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TopicSequence {
    pub epoch: u64,
    pub value: u64,
}

fn next_sequence(mut sequence: TopicSequence) -> TopicSequence {
    if sequence.value == u64::MAX {
        sequence.value = 0;
        sequence.epoch = sequence.epoch.wrapping_add(1);
    } else {
        sequence.value += 1;
    }
    sequence
}

fn sequence_distance_saturated(first: TopicSequence, last: TopicSequence) -> u64 {
    let epoch_distance = last.epoch.wrapping_sub(first.epoch);
    if epoch_distance == 0 {
        return last.value.wrapping_sub(first.value);
    }
    if epoch_distance != 1 || first.value == 0 {
        return u64::MAX;
    }
    let to_next_epoch = u64::MAX - first.value + 1;
    to_next_epoch.saturating_add(last.value)
}

fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// One worker-owned 64-bit event flag set for up to 64 subscribed topics.
#[derive(Default)]
pub struct TopicWaitSet {
    ready_topics: Mutex<u64>,
    condition: Condvar,
}

impl TopicWaitSet {
    pub const MAXIMUM_TOPICS: usize = 64;
    pub const ALL_TOPICS: u64 = u64::MAX;

    /// Converts a topic index into its event bit.
    pub fn bit(bit_index: u8) -> io::Result<u64> {
        if usize::from(bit_index) >= Self::MAXIMUM_TOPICS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "topic wait-set bit must be between 0 and 63",
            ));
        }
        Ok(1_u64 << bit_index)
    }

    /// Blocks without polling until one interested topic becomes ready.
    pub fn wait(&self, interested_topics: u64) -> io::Result<u64> {
        if interested_topics == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "wait-set topic mask cannot be zero",
            ));
        }
        let mut ready = lock_or_recover(&self.ready_topics);
        while *ready & interested_topics == 0 {
            ready = self
                .condition
                .wait(ready)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
        Ok(*ready & interested_topics)
    }

    /// Returns zero when the timeout expires before a topic is ready.
    pub fn wait_timeout(&self, interested_topics: u64, timeout: Duration) -> io::Result<u64> {
        if interested_topics == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "wait-set topic mask cannot be zero",
            ));
        }
        let deadline = Instant::now() + timeout;
        let mut ready = lock_or_recover(&self.ready_topics);
        loop {
            let matching = *ready & interested_topics;
            if matching != 0 {
                return Ok(matching);
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(0);
            }
            let (new_ready, result) = self
                .condition
                .wait_timeout(ready, deadline.saturating_duration_since(now))
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            ready = new_ready;
            if result.timed_out() && *ready & interested_topics == 0 {
                return Ok(0);
            }
        }
    }

    pub fn ready_topics(&self) -> u64 {
        *lock_or_recover(&self.ready_topics)
    }

    fn set_ready(&self, topics: u64) {
        *lock_or_recover(&self.ready_topics) |= topics;
        self.condition.notify_all();
    }

    fn clear_ready(&self, topics: u64) {
        *lock_or_recover(&self.ready_topics) &= !topics;
    }
}

struct RingSlot<T> {
    sequence: TopicSequence,
    message: Option<Arc<T>>,
}

struct SubscriberState {
    cursor: TopicSequence,
    read_index: usize,
    dropped_messages: u64,
    wait_set: Arc<TopicWaitSet>,
    ready_bit: u64,
    active: bool,
}

struct TopicState<T> {
    capacity: usize,
    ring: Vec<RingSlot<T>>,
    write_index: usize,
    retained_count: usize,
    next_write_sequence: TopicSequence,
    subscribers: Vec<Weak<Mutex<SubscriberState>>>,
    publisher_count: usize,
    closed: bool,
}

impl<T> TopicState<T> {
    fn oldest_index(&self) -> usize {
        if self.retained_count < self.capacity {
            0
        } else {
            self.write_index
        }
    }
}

/// Result of one blocking or non-blocking subscriber operation.
pub enum ReceiveStatus<T> {
    Message(Arc<T>),
    Timeout,
    Closed,
}

/// One bounded broadcast ring whose immutable messages are shared by subscribers.
pub struct Topic<T> {
    name: String,
    state: Arc<Mutex<TopicState<T>>>,
}

impl<T> Topic<T>
where
    T: Send + Sync + 'static,
{
    pub fn new(name: impl Into<String>, capacity: usize) -> io::Result<Self> {
        if capacity == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "topic capacity must be greater than zero",
            ));
        }
        let ring = (0..capacity)
            .map(|_| RingSlot {
                sequence: TopicSequence::default(),
                message: None,
            })
            .collect();
        Ok(Self {
            name: name.into(),
            state: Arc::new(Mutex::new(TopicState {
                capacity,
                ring,
                write_index: 0,
                retained_count: 0,
                next_write_sequence: TopicSequence::default(),
                subscribers: Vec::new(),
                publisher_count: 0,
                closed: false,
            })),
        })
    }

    pub fn publisher(&self) -> io::Result<TopicPublisher<T>> {
        let mut state = lock_or_recover(&self.state);
        if state.closed {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                format!("cannot create a publisher for closed topic '{}'", self.name),
            ));
        }
        state.publisher_count += 1;
        drop(state);
        Ok(TopicPublisher {
            name: self.name.clone(),
            state: Arc::clone(&self.state),
        })
    }

    /// New subscribers start at the next publication and skip retained history.
    pub fn subscribe(&self, subscriber_name: impl Into<String>) -> io::Result<TopicSubscriber<T>> {
        self.subscribe_with_wait_set(subscriber_name, Arc::new(TopicWaitSet::default()), 0)
    }

    pub fn subscribe_with_wait_set(
        &self,
        subscriber_name: impl Into<String>,
        wait_set: Arc<TopicWaitSet>,
        wait_bit: u8,
    ) -> io::Result<TopicSubscriber<T>> {
        let ready_bit = TopicWaitSet::bit(wait_bit)?;
        let subscriber = Arc::new(Mutex::new(SubscriberState {
            cursor: TopicSequence::default(),
            read_index: 0,
            dropped_messages: 0,
            wait_set,
            ready_bit,
            active: true,
        }));
        let mut state = lock_or_recover(&self.state);
        {
            let mut subscriber_state = lock_or_recover(&subscriber);
            subscriber_state.cursor = state.next_write_sequence;
            subscriber_state.read_index = state.write_index;
            if state.closed {
                subscriber_state.wait_set.set_ready(ready_bit);
            } else {
                state.subscribers.push(Arc::downgrade(&subscriber));
            }
        }
        drop(state);
        Ok(TopicSubscriber {
            topic_name: self.name.clone(),
            subscriber_name: subscriber_name.into(),
            state: Arc::clone(&self.state),
            subscriber,
        })
    }

    pub fn close(&self) {
        close_topic(&self.state);
    }
}

fn close_topic<T>(state: &Arc<Mutex<TopicState<T>>>) {
    let mut state = lock_or_recover(state);
    if state.closed {
        return;
    }
    state.closed = true;
    state.subscribers.retain(|weak| {
        if let Some(subscriber) = weak.upgrade() {
            let subscriber = lock_or_recover(&subscriber);
            if subscriber.active {
                subscriber.wait_set.set_ready(subscriber.ready_bit);
            }
            true
        } else {
            false
        }
    });
}

/// Publishing endpoint owned by one producer worker.
pub struct TopicPublisher<T> {
    name: String,
    state: Arc<Mutex<TopicState<T>>>,
}

impl<T> Clone for TopicPublisher<T> {
    fn clone(&self) -> Self {
        lock_or_recover(&self.state).publisher_count += 1;
        Self {
            name: self.name.clone(),
            state: Arc::clone(&self.state),
        }
    }
}

impl<T> TopicPublisher<T>
where
    T: Send + Sync + 'static,
{
    pub fn publish(&self, message: T) -> io::Result<usize> {
        self.publish_shared(Arc::new(message))
    }

    pub fn publish_shared(&self, message: Arc<T>) -> io::Result<usize> {
        let mut state = lock_or_recover(&self.state);
        if state.closed {
            return Ok(0);
        }
        let write_index = state.write_index;
        let sequence = state.next_write_sequence;
        state.ring[write_index].sequence = sequence;
        state.ring[write_index].message = Some(message);
        state.write_index = (write_index + 1) % state.capacity;
        state.retained_count = (state.retained_count + 1).min(state.capacity);
        state.next_write_sequence = next_sequence(sequence);

        let mut signalled = 0;
        state.subscribers.retain(|weak| {
            if let Some(subscriber) = weak.upgrade() {
                let subscriber = lock_or_recover(&subscriber);
                if subscriber.active {
                    subscriber.wait_set.set_ready(subscriber.ready_bit);
                    signalled += 1;
                }
                true
            } else {
                false
            }
        });
        Ok(signalled)
    }

    pub fn name(&self) -> &str {
        &self.name
    }
}

impl<T> Drop for TopicPublisher<T> {
    fn drop(&mut self) {
        let should_close = {
            let mut state = lock_or_recover(&self.state);
            if state.publisher_count > 0 {
                state.publisher_count -= 1;
            }
            state.publisher_count == 0 && !state.closed
        };
        if should_close {
            close_topic(&self.state);
        }
    }
}

/// Subscriber with its own cursor and dropped-message counter.
pub struct TopicSubscriber<T> {
    topic_name: String,
    subscriber_name: String,
    state: Arc<Mutex<TopicState<T>>>,
    subscriber: Arc<Mutex<SubscriberState>>,
}

impl<T> TopicSubscriber<T>
where
    T: Send + Sync + 'static,
{
    pub fn receive(&self) -> io::Result<ReceiveStatus<T>> {
        loop {
            let status = self.try_receive()?;
            if !matches!(status, ReceiveStatus::Timeout) {
                return Ok(status);
            }
            let subscriber = lock_or_recover(&self.subscriber);
            let wait_set = Arc::clone(&subscriber.wait_set);
            let ready_bit = subscriber.ready_bit;
            drop(subscriber);
            wait_set.wait(ready_bit)?;
        }
    }

    pub fn try_receive(&self) -> io::Result<ReceiveStatus<T>> {
        let state = lock_or_recover(&self.state);
        let mut subscriber = lock_or_recover(&self.subscriber);
        try_receive_locked(&state, &mut subscriber)
    }

    pub fn receive_timeout(&self, timeout: Duration) -> io::Result<ReceiveStatus<T>> {
        let deadline = Instant::now() + timeout;
        loop {
            let status = self.try_receive()?;
            if !matches!(status, ReceiveStatus::Timeout) {
                return Ok(status);
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(ReceiveStatus::Timeout);
            }
            let subscriber = lock_or_recover(&self.subscriber);
            let wait_set = Arc::clone(&subscriber.wait_set);
            let ready_bit = subscriber.ready_bit;
            drop(subscriber);
            if wait_set.wait_timeout(ready_bit, deadline.saturating_duration_since(now))? == 0 {
                return Ok(ReceiveStatus::Timeout);
            }
        }
    }

    pub fn dropped_messages(&self) -> u64 {
        lock_or_recover(&self.subscriber).dropped_messages
    }

    pub fn cursor(&self) -> TopicSequence {
        lock_or_recover(&self.subscriber).cursor
    }

    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    pub fn subscriber_name(&self) -> &str {
        &self.subscriber_name
    }
}

fn try_receive_locked<T>(
    state: &TopicState<T>,
    subscriber: &mut SubscriberState,
) -> io::Result<ReceiveStatus<T>> {
    if !subscriber.active {
        return Ok(ReceiveStatus::Closed);
    }
    if subscriber.cursor != state.next_write_sequence {
        let expected = &state.ring[subscriber.read_index];
        if expected.message.is_none() || expected.sequence != subscriber.cursor {
            let oldest_index = state.oldest_index();
            let oldest = &state.ring[oldest_index];
            if oldest.message.is_none() {
                return Err(io::Error::other("ring buffer has no retained message"));
            }
            let dropped = sequence_distance_saturated(subscriber.cursor, oldest.sequence);
            subscriber.dropped_messages = subscriber.dropped_messages.saturating_add(dropped);
            subscriber.cursor = oldest.sequence;
            subscriber.read_index = oldest_index;
        }

        let slot = &state.ring[subscriber.read_index];
        if slot.message.is_none() || slot.sequence != subscriber.cursor {
            return Err(io::Error::other("ring-buffer subscriber cursor mismatch"));
        }
        let message = Arc::clone(slot.message.as_ref().expect("checked above"));
        subscriber.cursor = next_sequence(subscriber.cursor);
        subscriber.read_index = (subscriber.read_index + 1) % state.capacity;
        if subscriber.cursor == state.next_write_sequence {
            subscriber.wait_set.clear_ready(subscriber.ready_bit);
        }
        return Ok(ReceiveStatus::Message(message));
    }

    subscriber.wait_set.clear_ready(subscriber.ready_bit);
    Ok(if state.closed {
        ReceiveStatus::Closed
    } else {
        ReceiveStatus::Timeout
    })
}

impl<T> Drop for TopicSubscriber<T> {
    fn drop(&mut self) {
        let _state = lock_or_recover(&self.state);
        let mut subscriber = lock_or_recover(&self.subscriber);
        subscriber.active = false;
        subscriber.wait_set.clear_ready(subscriber.ready_bit);
    }
}

#[cfg(test)]
#[path = "../../unittest/platform_test/pubsub_test.rs"]
mod tests;
