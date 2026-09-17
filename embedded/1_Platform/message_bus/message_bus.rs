//! Named, strongly typed topics shared by independent workers.

use std::{
    any::Any,
    collections::HashMap,
    io,
    sync::{Arc, Mutex, MutexGuard},
    time::Duration,
};

use super::pubsub::{ReceiveStatus, Topic, TopicPublisher, TopicSubscriber, TopicWaitSet};

fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

trait ErasedTopic: Send + Sync {
    fn as_any(&self) -> &dyn Any;
    fn close(&self);
}

struct TopicHolder<T>
where
    T: Send + Sync + 'static,
{
    topic: Arc<Topic<T>>,
}

impl<T> ErasedTopic for TopicHolder<T>
where
    T: Send + Sync + 'static,
{
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn close(&self) {
        self.topic.close();
    }
}

struct MessageBusState {
    capacities: HashMap<String, usize>,
    topics: HashMap<String, Arc<dyn ErasedTopic>>,
    closed: bool,
}

/// Owns named topics while each worker creates its own endpoints.
pub struct MessageBus {
    default_capacity: usize,
    state: Mutex<MessageBusState>,
}

impl MessageBus {
    pub fn new(default_capacity: usize) -> io::Result<Self> {
        if default_capacity == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "message bus default capacity must be greater than zero",
            ));
        }
        Ok(Self {
            default_capacity,
            state: Mutex::new(MessageBusState {
                capacities: HashMap::new(),
                topics: HashMap::new(),
                closed: false,
            }),
        })
    }

    pub fn configure_topic(&self, name: impl Into<String>, capacity: usize) -> io::Result<()> {
        if capacity == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "message bus topic capacity must be greater than zero",
            ));
        }
        let name = name.into();
        let mut state = lock_or_recover(&self.state);
        if state.closed {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "cannot configure a closed message bus",
            ));
        }
        if state.topics.contains_key(&name) {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                format!("cannot change capacity after topic '{name}' was created"),
            ));
        }
        state.capacities.insert(name, capacity);
        Ok(())
    }

    pub fn publisher<T>(&self, topic_name: &str) -> io::Result<TopicPublisher<T>>
    where
        T: Send + Sync + 'static,
    {
        self.topic::<T>(topic_name)?.publisher()
    }

    pub fn subscribe<T>(
        &self,
        topic_name: &str,
        subscriber_name: impl Into<String>,
    ) -> io::Result<TopicSubscriber<T>>
    where
        T: Send + Sync + 'static,
    {
        self.topic::<T>(topic_name)?.subscribe(subscriber_name)
    }

    pub fn subscribe_with_wait_set<T>(
        &self,
        topic_name: &str,
        subscriber_name: impl Into<String>,
        wait_set: Arc<TopicWaitSet>,
        wait_bit: u8,
    ) -> io::Result<TopicSubscriber<T>>
    where
        T: Send + Sync + 'static,
    {
        self.topic::<T>(topic_name)?
            .subscribe_with_wait_set(subscriber_name, wait_set, wait_bit)
    }

    pub fn close(&self) {
        let topics = {
            let mut state = lock_or_recover(&self.state);
            if state.closed {
                return;
            }
            state.closed = true;
            state.topics.values().cloned().collect::<Vec<_>>()
        };
        for topic in topics {
            topic.close();
        }
    }

    fn topic<T>(&self, name: &str) -> io::Result<Arc<Topic<T>>>
    where
        T: Send + Sync + 'static,
    {
        let mut state = lock_or_recover(&self.state);
        if state.closed {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                format!("cannot use closed message bus topic '{name}'"),
            ));
        }
        if let Some(existing) = state.topics.get(name) {
            let holder = existing
                .as_any()
                .downcast_ref::<TopicHolder<T>>()
                .ok_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("topic '{name}' was requested with a different message type"),
                    )
                })?;
            return Ok(Arc::clone(&holder.topic));
        }

        let capacity = state
            .capacities
            .get(name)
            .copied()
            .unwrap_or(self.default_capacity);
        let topic = Arc::new(Topic::new(name, capacity)?);
        state.topics.insert(
            name.to_owned(),
            Arc::new(TopicHolder {
                topic: Arc::clone(&topic),
            }),
        );
        Ok(topic)
    }
}

impl Drop for MessageBus {
    fn drop(&mut self) {
        self.close();
    }
}

/// One typed subscription associated with a worker-owned wait-set bit.
pub struct WorkerTopicInput<T>
where
    T: Send + Sync + 'static,
{
    subscriber: TopicSubscriber<T>,
    ready_bit: u64,
}

impl<T> WorkerTopicInput<T>
where
    T: Send + Sync + 'static,
{
    pub fn is_ready(&self, ready_topics: u64) -> bool {
        ready_topics & self.ready_bit != 0
    }

    pub fn ready_bit(&self) -> u64 {
        self.ready_bit
    }

    pub fn try_receive(&self) -> io::Result<ReceiveStatus<T>> {
        self.subscriber.try_receive()
    }

    pub fn receive(&self) -> io::Result<ReceiveStatus<T>> {
        self.subscriber.receive()
    }

    pub fn dropped_messages(&self) -> u64 {
        self.subscriber.dropped_messages()
    }

    pub fn into_subscriber(self) -> TopicSubscriber<T> {
        self.subscriber
    }
}

/// Automatically assigns one of 64 wait-set bits to each worker input topic.
pub struct WorkerTopicInputs {
    worker_name: String,
    wait_set: Arc<TopicWaitSet>,
    subscribed_topics: u64,
    next_bit: usize,
}

impl WorkerTopicInputs {
    pub fn new(worker_name: impl Into<String>) -> io::Result<Self> {
        let worker_name = worker_name.into();
        if worker_name.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "worker topic-input name cannot be empty",
            ));
        }
        Ok(Self {
            worker_name,
            wait_set: Arc::new(TopicWaitSet::default()),
            subscribed_topics: 0,
            next_bit: 0,
        })
    }

    pub fn subscribe<T>(
        &mut self,
        bus: &MessageBus,
        topic_name: &str,
    ) -> io::Result<WorkerTopicInput<T>>
    where
        T: Send + Sync + 'static,
    {
        if self.next_bit >= TopicWaitSet::MAXIMUM_TOPICS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "worker cannot subscribe to more than 64 topics",
            ));
        }
        let bit_index = self.next_bit as u8;
        let ready_bit = TopicWaitSet::bit(bit_index)?;
        let subscriber = bus.subscribe_with_wait_set(
            topic_name,
            self.worker_name.clone(),
            Arc::clone(&self.wait_set),
            bit_index,
        )?;
        self.next_bit += 1;
        self.subscribed_topics |= ready_bit;
        Ok(WorkerTopicInput {
            subscriber,
            ready_bit,
        })
    }

    pub fn wait(&self) -> io::Result<u64> {
        self.ensure_has_inputs()?;
        self.wait_set.wait(self.subscribed_topics)
    }

    pub fn wait_timeout(&self, timeout: Duration) -> io::Result<u64> {
        self.ensure_has_inputs()?;
        self.wait_set.wait_timeout(self.subscribed_topics, timeout)
    }

    pub fn ready_topics(&self) -> u64 {
        self.wait_set.ready_topics() & self.subscribed_topics
    }

    pub fn topic_count(&self) -> usize {
        self.next_bit
    }

    fn ensure_has_inputs(&self) -> io::Result<()> {
        if self.subscribed_topics == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "worker has no subscribed input topics",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../unittest/platform_test/message_bus_test.rs"]
mod tests;
