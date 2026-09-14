//! Small in-process publish/subscribe topics backed by bounded Rust channels.

use std::{
    io,
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc::{self, Receiver, RecvError, RecvTimeoutError, SyncSender},
        Arc, Mutex,
    },
    time::Duration,
};

/// Creates publishers and independent bounded queues for one strongly typed topic.
pub struct Topic<T> {
    name: &'static str,
    capacity: usize,
    state: Arc<TopicState<T>>,
}

struct TopicState<T> {
    subscribers: Mutex<Vec<SyncSender<Arc<T>>>>,
    publisher_count: AtomicUsize,
}

impl<T> Topic<T> {
    /// Creates a topic whose subscriber queues cannot grow without limit.
    pub fn new(name: &'static str, capacity: usize) -> io::Result<Self> {
        if capacity == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("topic '{name}' capacity must be greater than zero"),
            ));
        }

        Ok(Self {
            name,
            capacity,
            state: Arc::new(TopicState {
                subscribers: Mutex::new(Vec::new()),
                publisher_count: AtomicUsize::new(0),
            }),
        })
    }

    /// Returns a publishing handle for the producer that owns this topic.
    pub fn publisher(&self) -> TopicPublisher<T> {
        self.state.publisher_count.fetch_add(1, Ordering::Relaxed);
        TopicPublisher {
            name: self.name,
            state: Arc::clone(&self.state),
        }
    }

    /// Adds one subscriber with its own queue, independent from other subscribers.
    pub fn subscribe(&self, subscriber_name: &'static str) -> io::Result<TopicSubscriber<T>> {
        let (sender, receiver) = mpsc::sync_channel(self.capacity);
        self.state
            .subscribers
            .lock()
            .map_err(|_| io::Error::other(format!("topic '{}' is poisoned", self.name)))?
            .push(sender);

        Ok(TopicSubscriber {
            topic_name: self.name,
            subscriber_name,
            receiver,
        })
    }
}

/// Publishes one shared message to every subscriber queue.
pub struct TopicPublisher<T> {
    name: &'static str,
    state: Arc<TopicState<T>>,
}

impl<T> Clone for TopicPublisher<T> {
    fn clone(&self) -> Self {
        self.state.publisher_count.fetch_add(1, Ordering::Relaxed);
        Self {
            name: self.name,
            state: Arc::clone(&self.state),
        }
    }
}

impl<T> TopicPublisher<T> {
    /// Wraps an owned message in `Arc` and broadcasts it without copying its payload.
    pub fn publish(&self, message: T) -> io::Result<usize> {
        self.publish_shared(Arc::new(message))
    }

    /// Broadcasts an already shared message and removes disconnected subscribers.
    pub fn publish_shared(&self, message: Arc<T>) -> io::Result<usize> {
        let mut subscribers = self
            .state
            .subscribers
            .lock()
            .map_err(|_| io::Error::other(format!("topic '{}' is poisoned", self.name)))?;
        let mut delivered = 0;
        let mut index = 0;

        // `send` intentionally blocks when a reliable bounded queue is full.
        // This prevents silent RAW or PCD data loss and keeps memory bounded.
        while index < subscribers.len() {
            if subscribers[index].send(Arc::clone(&message)).is_ok() {
                delivered += 1;
                index += 1;
            } else {
                subscribers.remove(index);
            }
        }
        Ok(delivered)
    }

    /// Returns the stable name used in logs and architecture documentation.
    pub fn name(&self) -> &'static str {
        self.name
    }
}

impl<T> Drop for TopicPublisher<T> {
    fn drop(&mut self) {
        if self.state.publisher_count.fetch_sub(1, Ordering::AcqRel) == 1 {
            // Closing every sender wakes subscribers after the final publisher exits.
            match self.state.subscribers.lock() {
                Ok(mut subscribers) => subscribers.clear(),
                Err(poisoned) => poisoned.into_inner().clear(),
            }
        }
    }
}

/// Receives every message published after this subscription was created.
pub struct TopicSubscriber<T> {
    topic_name: &'static str,
    subscriber_name: &'static str,
    receiver: Receiver<Arc<T>>,
}

impl<T> TopicSubscriber<T> {
    /// Blocks without consuming CPU until data arrives or all publishers close.
    pub fn recv(&self) -> Result<Arc<T>, RecvError> {
        self.receiver.recv()
    }

    /// Waits for a bounded time so a worker can periodically observe shutdown state.
    pub fn recv_timeout(&self, timeout: Duration) -> Result<Arc<T>, RecvTimeoutError> {
        self.receiver.recv_timeout(timeout)
    }

    /// Returns the topic name for diagnostics.
    pub fn topic_name(&self) -> &'static str {
        self.topic_name
    }

    /// Returns the worker name attached to this subscription.
    pub fn subscriber_name(&self) -> &'static str {
        self.subscriber_name
    }
}

#[cfg(test)]
#[path = "../unittest/platform_test/pubsub_test.rs"]
mod tests;
