//! Shared worker lifecycle helpers; main still chooses every worker explicitly.

use std::{
    io,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use super::{
    install_shutdown_signal_handlers,
    message_bus::MessageBus,
    shutdown_signal_received,
    threading::{PriorityStatus, StopToken, WorkerHandle},
};

/// Stores the report or error returned asynchronously by one worker.
#[derive(Debug, Default)]
pub struct WorkerResult<R> {
    pub report: Option<R>,
    pub error: Option<String>,
}

pub type SharedWorkerResult<R> = Arc<Mutex<WorkerResult<R>>>;

pub fn new_worker_result<R>() -> SharedWorkerResult<R> {
    Arc::new(Mutex::new(WorkerResult {
        report: None,
        error: None,
    }))
}

/// Creates the completion callback supplied to one spawn function.
pub fn completion_for<R>(
    result: SharedWorkerResult<R>,
) -> impl FnOnce(Result<R, String>) + Send + 'static
where
    R: Send + 'static,
{
    move |completion| {
        let mut output = result
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        match completion {
            Ok(report) => output.report = Some(report),
            Err(error) => output.error = Some(error),
        }
    }
}

/// Prints native priority status and records a newly started worker.
pub fn add_worker(workers: &mut Vec<WorkerHandle<()>>, handle: WorkerHandle<()>) {
    match handle.priority_status() {
        PriorityStatus::Applied { requested, native } => println!(
            "Thread '{}': priority {} -> {} {}",
            handle.name(),
            requested.level(),
            native.policy,
            native.value
        ),
        PriorityStatus::Unchanged { requested, error } => eprintln!(
            "Warning: thread '{}' kept inherited priority (requested {}): {}",
            handle.name(),
            requested.level(),
            error
        ),
    }
    workers.push(handle);
}

/// Waits until Ctrl+C/SIGTERM or a worker requests cooperative shutdown.
pub fn wait_for_shutdown_request(stop: &StopToken) -> io::Result<()> {
    install_shutdown_signal_handlers()?;
    while !stop.is_stop_requested() && !shutdown_signal_received() {
        thread::sleep(Duration::from_millis(100));
    }
    stop.request_stop();
    Ok(())
}

/// Requests shutdown, closes the bus, and joins without hiding a startup error.
pub fn stop_and_join_noexcept(
    workers: &mut Vec<WorkerHandle<()>>,
    stop: &StopToken,
    bus: &MessageBus,
) {
    stop.request_stop();
    bus.close();
    for worker in workers.drain(..) {
        let _ = worker.join();
    }
}

/// Joins all workers and records thread panics.
pub fn join_workers(
    workers: &mut Vec<WorkerHandle<()>>,
    stop: &StopToken,
    bus: &MessageBus,
    failures: &mut Vec<String>,
) {
    for worker in workers.drain(..) {
        let name = worker.name().to_owned();
        if worker.join().is_err() {
            failures.push(format!("{name}: worker panicked"));
            stop.request_stop();
            bus.close();
        }
    }
}

/// Appends the completion error for a worker that was actually started.
pub fn collect_worker_error<R>(
    failures: &mut Vec<String>,
    name: &str,
    result: &SharedWorkerResult<R>,
    was_started: bool,
) {
    if !was_started {
        return;
    }
    let result = result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    if let Some(error) = &result.error {
        failures.push(format!("{name}: {error}"));
    } else if result.report.is_none() {
        failures.push(format!("{name}: worker exited without a completion report"));
    }
}

pub fn failure_if_any(failures: Vec<String>) -> io::Result<()> {
    if failures.is_empty() {
        Ok(())
    } else {
        Err(io::Error::other(failures.join("; ")))
    }
}

#[cfg(test)]
#[path = "../../unittest/platform_test/workers_test.rs"]
mod tests;
