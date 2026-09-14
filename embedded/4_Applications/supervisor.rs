//! Controls worker lifecycle while data moves independently through Pub/Sub topics.

use std::{
    collections::HashSet,
    io,
    path::PathBuf,
    sync::mpsc,
    time::{Duration, Instant},
};

use crate::{
    application::{
        logger::{spawn_pcd_logger, spawn_raw_logger, LoggerReport},
        pointcloud_processing::{
            spawn_preprocessing_worker, PreprocessingConfig, PreprocessingReport,
        },
    },
    devices::lidar::{
        spawn_lidar_decode_worker, spawn_lidar_read_worker, Lidar, LidarPointCloudMessage,
        LidarRawMessage, LidarWorkerReport,
    },
    platform::{
        pubsub::Topic,
        threading::{PriorityStatus, StopToken, ThreadConfig, WorkerHandle},
    },
};

/// Stable topic names shared by all current and future LiDAR drivers.
pub const LIDAR_RAW_TOPIC: &str = "lidar/raw";
pub const POINTCLOUD_DECODED_TOPIC: &str = "pointcloud/decoded";
pub const POINTCLOUD_PROCESSED_TOPIC: &str = "pointcloud/processed";

/// Numeric priority and OS-thread name for every current capture worker.
#[derive(Debug, Clone)]
pub struct CaptureThreadConfig {
    pub lidar_read: ThreadConfig,
    pub lidar_decode: ThreadConfig,
    pub raw_logger: ThreadConfig,
    pub preprocessing: ThreadConfig,
    pub pcd_logger: ThreadConfig,
}

/// Maximum number of pending messages allowed on each in-process topic.
#[derive(Debug, Clone, Copy)]
pub struct TopicQueueConfig {
    pub raw_capacity: usize,
    pub decoded_capacity: usize,
    pub processed_capacity: usize,
}

/// Complete input supplied by `config.rs` to one supervised capture session.
#[derive(Debug, Clone)]
pub struct CaptureConfig {
    pub duration: Duration,
    pub raw_path: Option<PathBuf>,
    pub pointcloud_path: Option<PathBuf>,
    pub preprocessing: PreprocessingConfig,
    pub threads: CaptureThreadConfig,
    pub queues: TopicQueueConfig,
}

/// Summarizes the amount of data produced by one capture session.
#[derive(Debug, Clone, Copy)]
pub struct CaptureStats {
    pub packet_count: u64,
    pub point_count: u64,
    pub elapsed: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum StageKind {
    LidarRead,
    LidarDecode,
    RawLogger,
    Preprocessing,
    PcdLogger,
}

#[derive(Debug, Default)]
struct StageReport {
    message_count: u64,
    point_count: u64,
}

impl From<LidarWorkerReport> for StageReport {
    /// Converts device worker counters to the supervisor's common report.
    fn from(report: LidarWorkerReport) -> Self {
        Self {
            message_count: report.message_count,
            point_count: 0,
        }
    }
}

impl From<PreprocessingReport> for StageReport {
    /// Converts preprocessing counters to the supervisor's common report.
    fn from(report: PreprocessingReport) -> Self {
        Self {
            message_count: report.message_count,
            point_count: report.point_count,
        }
    }
}

impl From<LoggerReport> for StageReport {
    /// Converts logger counters to the supervisor's common report.
    fn from(report: LoggerReport) -> Self {
        Self {
            message_count: report.message_count,
            point_count: report.point_count,
        }
    }
}

struct StageCompletion {
    kind: StageKind,
    name: String,
    result: Result<StageReport, String>,
}

struct RunningWorker {
    kind: StageKind,
    handle: WorkerHandle<()>,
}

/// Wires independent workers together and supervises them until capture completes.
pub fn run_capture(lidar: Lidar, config: CaptureConfig) -> io::Result<CaptureStats> {
    if config.duration.is_zero() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "capture duration must be greater than zero",
        ));
    }

    let started = Instant::now();
    let lidar = lidar.into_parts();
    let lidar_id = lidar.lidar_id.to_owned();
    let stop = StopToken::new();
    let (completion_sender, completion_receiver) = mpsc::channel();

    // The supervisor owns wiring only; worker modules own all data processing.
    let raw_topic: Topic<LidarRawMessage> =
        Topic::new(LIDAR_RAW_TOPIC, config.queues.raw_capacity)?;
    let decoded_topic: Topic<LidarPointCloudMessage> =
        Topic::new(POINTCLOUD_DECODED_TOPIC, config.queues.decoded_capacity)?;
    let processed_topic: Topic<LidarPointCloudMessage> =
        Topic::new(POINTCLOUD_PROCESSED_TOPIC, config.queues.processed_capacity)?;

    let raw_decoder_subscription = raw_topic.subscribe("lidar-decode")?;
    let raw_logger_subscription = config
        .raw_path
        .as_ref()
        .map(|_| raw_topic.subscribe("raw-logger"))
        .transpose()?;
    let decoded_subscription = decoded_topic.subscribe("pointcloud-preprocessing")?;
    let processed_logger_subscription = config
        .pointcloud_path
        .as_ref()
        .map(|_| processed_topic.subscribe("pcd-logger"))
        .transpose()?;

    let raw_publisher = raw_topic.publisher();
    let decoded_publisher = decoded_topic.publisher();
    let processed_publisher = processed_topic.publisher();
    let mut workers = Vec::new();

    // Consumers start first so the initial sensor message always has a route.
    if let (Some(path), Some(subscription)) = (
        config.pointcloud_path.clone(),
        processed_logger_subscription,
    ) {
        let kind = StageKind::PcdLogger;
        let thread_config = config.threads.pcd_logger.clone();
        let completion = stage_completion::<LoggerReport>(
            kind,
            thread_config.name.clone(),
            completion_sender.clone(),
        );
        let handle = spawn_pcd_logger(thread_config, stop.clone(), subscription, path, completion)?;
        workers.push(register_worker(kind, handle));
    }

    let kind = StageKind::Preprocessing;
    let thread_config = config.threads.preprocessing.clone();
    let completion = stage_completion::<PreprocessingReport>(
        kind,
        thread_config.name.clone(),
        completion_sender.clone(),
    );
    let handle = spawn_preprocessing_worker(
        thread_config,
        stop.clone(),
        decoded_subscription,
        processed_publisher,
        config.preprocessing,
        completion,
    )?;
    workers.push(register_worker(kind, handle));

    let kind = StageKind::LidarDecode;
    let thread_config = config.threads.lidar_decode.clone();
    let completion = stage_completion::<LidarWorkerReport>(
        kind,
        thread_config.name.clone(),
        completion_sender.clone(),
    );
    let handle = spawn_lidar_decode_worker(
        thread_config,
        stop.clone(),
        lidar.decoder,
        raw_decoder_subscription,
        decoded_publisher,
        completion,
    )?;
    workers.push(register_worker(kind, handle));

    if let (Some(path), Some(subscription)) = (config.raw_path.clone(), raw_logger_subscription) {
        let kind = StageKind::RawLogger;
        let thread_config = config.threads.raw_logger.clone();
        let completion = stage_completion::<LoggerReport>(
            kind,
            thread_config.name.clone(),
            completion_sender.clone(),
        );
        let handle = spawn_raw_logger(thread_config, stop.clone(), subscription, path, completion)?;
        workers.push(register_worker(kind, handle));
    }

    let kind = StageKind::LidarRead;
    let thread_config = config.threads.lidar_read;
    let completion = stage_completion::<LidarWorkerReport>(
        kind,
        thread_config.name.clone(),
        completion_sender.clone(),
    );
    let handle = spawn_lidar_read_worker(
        thread_config,
        stop.clone(),
        lidar.reader,
        raw_publisher,
        lidar_id,
        config.duration,
        completion,
    )?;
    workers.push(register_worker(kind, handle));

    drop(completion_sender);
    collect_results(workers, completion_receiver, stop, started)
}

/// Adapts a module-specific report to the supervisor's completion channel.
fn stage_completion<R>(
    kind: StageKind,
    name: String,
    completion_sender: mpsc::Sender<StageCompletion>,
) -> impl FnOnce(Result<R, String>) + Send + 'static
where
    R: Into<StageReport> + Send + 'static,
{
    move |result| {
        let _ = completion_sender.send(StageCompletion {
            kind,
            name,
            result: result.map(Into::into),
        });
    }
}

/// Records a new worker after showing the OS priority applied at startup.
fn register_worker(kind: StageKind, handle: WorkerHandle<()>) -> RunningWorker {
    print_priority_status(&handle);
    RunningWorker { kind, handle }
}

/// Makes the effective native priority visible without failing on OS restrictions.
fn print_priority_status(handle: &WorkerHandle<()>) {
    match handle.priority_status() {
        PriorityStatus::Applied { requested, native } => println!(
            "Thread '{}': priority {} -> {} {}",
            handle.name(),
            requested.level(),
            native.policy,
            native.value
        ),
        PriorityStatus::Unchanged { requested, error } => eprintln!(
            "Warning: thread '{}' kept its inherited priority (requested {}): {}",
            handle.name(),
            requested.level(),
            error
        ),
    }
}

/// Waits for every worker, propagates failures, and returns capture statistics.
fn collect_results(
    workers: Vec<RunningWorker>,
    completion_receiver: mpsc::Receiver<StageCompletion>,
    stop: StopToken,
    started: Instant,
) -> io::Result<CaptureStats> {
    let expected_completions = workers.len();
    let mut completed_stages = HashSet::with_capacity(expected_completions);
    let mut packet_count = 0;
    let mut point_count = 0;
    let mut failures = Vec::new();

    while completed_stages.len() < expected_completions {
        match completion_receiver.recv_timeout(Duration::from_millis(100)) {
            Ok(completion) => {
                completed_stages.insert(completion.kind);
                match completion.result {
                    Ok(report) => match completion.kind {
                        StageKind::LidarRead => packet_count = report.message_count,
                        StageKind::Preprocessing => point_count = report.point_count,
                        _ => {}
                    },
                    Err(error) => failures.push(format!("{}: {error}", completion.name)),
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // A panic bypasses the normal callback, so detect it explicitly.
                for worker in &workers {
                    if worker.handle.is_finished() && !completed_stages.contains(&worker.kind) {
                        completed_stages.insert(worker.kind);
                        failures.push(format!("{}: worker panicked", worker.handle.name()));
                        stop.request_stop();
                    }
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                failures.push("worker completion channel closed unexpectedly".to_owned());
                stop.request_stop();
                break;
            }
        }
    }

    for worker in workers {
        let name = worker.handle.name().to_owned();
        if worker.handle.join().is_err()
            && !failures.iter().any(|failure| failure.starts_with(&name))
        {
            failures.push(format!("{name}: worker panicked"));
        }
    }

    if failures.is_empty() {
        Ok(CaptureStats {
            packet_count,
            point_count,
            elapsed: started.elapsed(),
        })
    } else {
        Err(io::Error::other(failures.join("; ")))
    }
}

#[cfg(test)]
#[path = "../unittest/application_test/supervisor_test.rs"]
mod tests;
