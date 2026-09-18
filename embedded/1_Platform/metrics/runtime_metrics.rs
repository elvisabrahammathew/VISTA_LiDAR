//! Process-wide counters used by monitoring without coupling workers together.

use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicU64, Ordering},
        Mutex, MutexGuard, OnceLock,
    },
    time::Instant,
};

static RAW_BYTES_WRITTEN: AtomicU64 = AtomicU64::new(0);
static RAW_MESSAGES_WRITTEN: AtomicU64 = AtomicU64::new(0);
static PCD_POINTS_WRITTEN: AtomicU64 = AtomicU64::new(0);
static PCD_FRAMES_WRITTEN: AtomicU64 = AtomicU64::new(0);
static STORAGE_WRITE_ERRORS: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Clone, Copy, Default)]
pub struct StorageMetricsSnapshot {
    pub raw_bytes_written: u64,
    pub raw_messages_written: u64,
    pub pcd_points_written: u64,
    pub pcd_frames_written: u64,
    pub write_errors: u64,
}

pub fn record_raw_write(bytes: usize) {
    RAW_BYTES_WRITTEN.fetch_add(bytes as u64, Ordering::Relaxed);
    RAW_MESSAGES_WRITTEN.fetch_add(1, Ordering::Relaxed);
}

pub fn record_pcd_write(points: usize) {
    PCD_POINTS_WRITTEN.fetch_add(points as u64, Ordering::Relaxed);
    PCD_FRAMES_WRITTEN.fetch_add(1, Ordering::Relaxed);
}

pub fn record_storage_write_error() {
    STORAGE_WRITE_ERRORS.fetch_add(1, Ordering::Relaxed);
}

pub fn storage_metrics_snapshot() -> StorageMetricsSnapshot {
    StorageMetricsSnapshot {
        raw_bytes_written: RAW_BYTES_WRITTEN.load(Ordering::Relaxed),
        raw_messages_written: RAW_MESSAGES_WRITTEN.load(Ordering::Relaxed),
        pcd_points_written: PCD_POINTS_WRITTEN.load(Ordering::Relaxed),
        pcd_frames_written: PCD_FRAMES_WRITTEN.load(Ordering::Relaxed),
        write_errors: STORAGE_WRITE_ERRORS.load(Ordering::Relaxed),
    }
}

#[derive(Debug, Clone)]
pub struct WorkerRuntimeSnapshot {
    pub name: String,
    pub priority: u8,
    pub running: bool,
    pub failed: bool,
    pub uptime_seconds: f64,
}

struct WorkerRuntimeState {
    priority: u8,
    started: Instant,
    running: bool,
    failed: bool,
}

fn worker_states() -> &'static Mutex<HashMap<String, WorkerRuntimeState>> {
    static STATES: OnceLock<Mutex<HashMap<String, WorkerRuntimeState>>> = OnceLock::new();
    STATES.get_or_init(|| Mutex::new(HashMap::new()))
}

fn lock_states() -> MutexGuard<'static, HashMap<String, WorkerRuntimeState>> {
    worker_states()
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// RAII guard created inside each worker thread.
pub struct WorkerRuntimeGuard {
    name: String,
}

impl WorkerRuntimeGuard {
    pub fn started(name: impl Into<String>, priority: u8) -> Self {
        let name = name.into();
        lock_states().insert(
            name.clone(),
            WorkerRuntimeState {
                priority,
                started: Instant::now(),
                running: true,
                failed: false,
            },
        );
        Self { name }
    }
}

impl Drop for WorkerRuntimeGuard {
    fn drop(&mut self) {
        if let Some(state) = lock_states().get_mut(&self.name) {
            state.running = false;
            state.failed = std::thread::panicking();
        }
    }
}

pub fn worker_runtime_snapshot() -> Vec<WorkerRuntimeSnapshot> {
    let now = Instant::now();
    lock_states()
        .iter()
        .map(|(name, state)| WorkerRuntimeSnapshot {
            name: name.clone(),
            priority: state.priority,
            running: state.running,
            failed: state.failed,
            uptime_seconds: now.duration_since(state.started).as_secs_f64(),
        })
        .collect()
}
