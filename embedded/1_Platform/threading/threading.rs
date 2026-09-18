//! Cross-platform worker creation with project priority levels 1 through 5.

use std::{
    io,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    thread::{self, JoinHandle},
};

use super::{apply_current_thread_priority, runtime_metrics::WorkerRuntimeGuard};

pub const HIGHEST_PRIORITY: u8 = 1;
pub const LOWEST_PRIORITY: u8 = 5;

/// Cooperative shutdown flag shared by the supervisor and every worker.
///
/// A worker checks this token between blocking operations. Requesting a stop
/// does not forcibly terminate a thread, so device and file resources still
/// leave scope normally and are closed by Rust.
#[derive(Debug, Clone, Default)]
pub struct StopToken {
    requested: Arc<AtomicBool>,
}

impl StopToken {
    /// Creates a stop flag whose initial state allows workers to run.
    pub fn new() -> Self {
        Self::default()
    }

    /// Asks every clone of this token to finish as soon as it is safe.
    pub fn request_stop(&self) {
        self.requested.store(true, Ordering::Release);
    }

    /// Reports whether the supervisor or another worker requested shutdown.
    pub fn is_stop_requested(&self) -> bool {
        self.requested.load(Ordering::Acquire)
    }
}

/// Project-level priority where 1 is highest and 5 is lowest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThreadPriority(u8);

impl ThreadPriority {
    /// Validates a numeric priority before any thread is created.
    pub fn new(level: u8) -> io::Result<Self> {
        if !(HIGHEST_PRIORITY..=LOWEST_PRIORITY).contains(&level) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "thread priority must be between {HIGHEST_PRIORITY} (highest) and \
                     {LOWEST_PRIORITY} (lowest); received {level}"
                ),
            ));
        }
        Ok(Self(level))
    }

    /// Returns the user-facing numeric level configured in main.rs.
    pub fn level(self) -> u8 {
        self.0
    }
}

/// Values supplied by main.rs when starting one long-lived worker.
#[derive(Debug, Clone)]
pub struct ThreadConfig {
    pub name: String,
    pub priority: ThreadPriority,
}

impl ThreadConfig {
    /// Creates a named thread configuration using a validated priority.
    pub fn new(name: impl Into<String>, priority: u8) -> io::Result<Self> {
        Ok(Self {
            name: name.into(),
            priority: ThreadPriority::new(priority)?,
        })
    }
}

/// Native value read back after applying a project-level priority.
#[derive(Debug, Clone)]
pub struct NativeThreadPriority {
    pub policy: &'static str,
    pub value: i32,
}

/// Records whether the OS accepted the requested priority.
#[derive(Debug, Clone)]
pub enum PriorityStatus {
    Applied {
        requested: ThreadPriority,
        native: NativeThreadPriority,
    },
    Unchanged {
        requested: ThreadPriority,
        error: String,
    },
}

/// Owns a running worker and the priority result reported at startup.
pub struct WorkerHandle<T> {
    name: String,
    priority_status: PriorityStatus,
    join_handle: JoinHandle<T>,
}

impl<T> WorkerHandle<T> {
    /// Returns the configured worker name used by the supervisor.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the OS priority result captured before the worker loop started.
    pub fn priority_status(&self) -> &PriorityStatus {
        &self.priority_status
    }

    /// Reports whether the OS thread has already returned.
    pub fn is_finished(&self) -> bool {
        self.join_handle.is_finished()
    }

    /// Waits for thread cleanup and propagates any panic to the supervisor.
    pub fn join(self) -> thread::Result<T> {
        self.join_handle.join()
    }
}

/// Creates a named OS thread and applies its priority before executing work.
pub fn spawn_worker<T, F>(config: ThreadConfig, worker: F) -> io::Result<WorkerHandle<T>>
where
    T: Send + 'static,
    F: FnOnce() -> T + Send + 'static,
{
    let name = config.name.clone();
    let runtime_name = name.clone();
    let requested = config.priority;
    let (startup_sender, startup_receiver) = mpsc::sync_channel(1);

    let join_handle = thread::Builder::new().name(name.clone()).spawn(move || {
        let status = match apply_current_thread_priority(requested) {
            Ok(native) => PriorityStatus::Applied { requested, native },
            Err(error) => PriorityStatus::Unchanged {
                requested,
                error: error.to_string(),
            },
        };
        // The worker may continue at the inherited priority if the OS rejects
        // the request; main receives a visible warning through this handshake.
        let _ = startup_sender.send(status);
        let _runtime = WorkerRuntimeGuard::started(runtime_name, requested.level());
        worker()
    })?;

    let priority_status = startup_receiver.recv().map_err(|_| {
        io::Error::other(format!(
            "worker '{name}' exited before reporting its priority"
        ))
    })?;

    Ok(WorkerHandle {
        name,
        priority_status,
        join_handle,
    })
}

#[cfg(test)]
#[path = "../../unittest/platform_test/threading_test.rs"]
mod tests;
