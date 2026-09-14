//! Application workers that persist subscribed LiDAR data to local files.

use std::{io, path::PathBuf};

use crate::{
    devices::lidar::RawPacket,
    models::{lidar_message::LidarMessage, pointcloud::PointCloudFrame},
    platform::{
        pubsub::TopicSubscriber,
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
    },
    transport::local::{PcdWriter, RawCaptureWriter},
};

/// Counts the messages and points written by one logger worker.
#[derive(Debug, Default)]
pub(crate) struct LoggerReport {
    pub message_count: u64,
    pub point_count: u64,
}

/// Starts the RAW logger worker when a `.bin` output has been enabled.
pub(crate) fn spawn_raw_logger<F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    subscriber: TopicSubscriber<LidarMessage<RawPacket>>,
    path: PathBuf,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LoggerReport, String>) + Send + 'static,
{
    spawn_logger(thread_config, stop, on_complete, move || {
        run_raw_logger(subscriber, path)
    })
}

/// Starts the PCD logger worker when a `.pcd` output has been enabled.
pub(crate) fn spawn_pcd_logger<F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    subscriber: TopicSubscriber<LidarMessage<PointCloudFrame>>,
    path: PathBuf,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LoggerReport, String>) + Send + 'static,
{
    spawn_logger(thread_config, stop, on_complete, move || {
        run_pcd_logger(subscriber, path)
    })
}

/// Applies common shutdown and completion behavior around a logger loop.
fn spawn_logger<W, F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    on_complete: F,
    worker: W,
) -> io::Result<WorkerHandle<()>>
where
    W: FnOnce() -> io::Result<LoggerReport> + Send + 'static,
    F: FnOnce(Result<LoggerReport, String>) + Send + 'static,
{
    spawn_worker(thread_config, move || {
        let result = worker().map_err(|error| error.to_string());
        if result.is_err() {
            // A file error stops upstream producers before their queues fill.
            stop.request_stop();
        }
        on_complete(result);
    })
}

/// Drains the RAW topic and preserves each packet's original bytes.
fn run_raw_logger(
    subscriber: TopicSubscriber<LidarMessage<RawPacket>>,
    path: PathBuf,
) -> io::Result<LoggerReport> {
    let mut writer = RawCaptureWriter::create(&path)?;
    let mut report = LoggerReport::default();

    while let Ok(message) = subscriber.recv() {
        writer.write_packet(&message.payload)?;
        report.message_count += 1;
    }
    writer.finish()?;
    Ok(report)
}

/// Drains the processed point-cloud topic and finalizes one PCD file.
fn run_pcd_logger(
    subscriber: TopicSubscriber<LidarMessage<PointCloudFrame>>,
    path: PathBuf,
) -> io::Result<LoggerReport> {
    let mut writer = PcdWriter::create(&path)?;
    let mut report = LoggerReport::default();

    while let Ok(message) = subscriber.recv() {
        writer.write_frame(&message.payload)?;
        report.message_count += 1;
        report.point_count += message.payload.points.len() as u64;
    }

    let written_points = writer.finish()?;
    debug_assert_eq!(written_points, report.point_count);
    Ok(report)
}

#[cfg(test)]
#[path = "../../unittest/application_test/logger_test.rs"]
mod tests;
