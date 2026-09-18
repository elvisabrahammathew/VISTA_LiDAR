//! Application workers that persist subscribed LiDAR data to local files.

use std::{io, path::PathBuf, sync::Arc};

use crate::{
    devices::lidar::RawPacket,
    models::{lidar_message::LidarMessage, pointcloud::PointCloudFrame, topics},
    platform::{
        message_bus::{MessageBus, WorkerTopicInputs},
        pubsub::{ReceiveStatus, TopicSubscriber},
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
    },
    transport::local::{PcdWriter, RawCaptureWriter},
};

/// Counts the messages and points written by one logger worker.
#[derive(Debug, Default)]
pub struct LoggerReport {
    pub message_count: u64,
    pub point_count: u64,
    pub dropped_message_count: u64,
}

/// Starts the RAW logger worker when a `.bin` output has been enabled.
pub fn spawn_raw_logger<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    path: PathBuf,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LoggerReport, String>) + Send + 'static,
{
    let mut inputs = WorkerTopicInputs::new(thread_config.name.clone())?;
    let subscriber = inputs
        .subscribe::<LidarMessage<RawPacket>>(&bus, topics::LIDAR_RAW)?
        .into_subscriber();
    spawn_logger(thread_config, stop, on_complete, move || {
        run_raw_logger(subscriber, path)
    })
}

/// Starts the PCD logger worker when a `.pcd` output has been enabled.
pub fn spawn_pcd_logger<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    path: PathBuf,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LoggerReport, String>) + Send + 'static,
{
    let mut inputs = WorkerTopicInputs::new(thread_config.name.clone())?;
    let subscriber = inputs
        .subscribe::<LidarMessage<PointCloudFrame>>(&bus, topics::POINTCLOUD_PROCESSED)?
        .into_subscriber();
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

    loop {
        match subscriber.receive()? {
            ReceiveStatus::Message(message) => {
                if let Err(error) = writer.write_packet(&message.payload) {
                    crate::platform::runtime_metrics::record_storage_write_error();
                    return Err(error);
                }
                crate::platform::runtime_metrics::record_raw_write(message.payload.len());
                report.message_count += 1;
            }
            ReceiveStatus::Closed => break,
            ReceiveStatus::Timeout => continue,
        }
    }
    report.dropped_message_count = subscriber.dropped_messages();
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

    loop {
        match subscriber.receive()? {
            ReceiveStatus::Message(message) => {
                if let Err(error) = writer.write_frame(&message.payload) {
                    crate::platform::runtime_metrics::record_storage_write_error();
                    return Err(error);
                }
                crate::platform::runtime_metrics::record_pcd_write(message.payload.points.len());
                report.message_count += 1;
                report.point_count += message.payload.points.len() as u64;
            }
            ReceiveStatus::Closed => break,
            ReceiveStatus::Timeout => continue,
        }
    }
    report.dropped_message_count = subscriber.dropped_messages();

    let written_points = writer.finish()?;
    debug_assert_eq!(written_points, report.point_count);
    Ok(report)
}

#[cfg(test)]
#[path = "../../unittest/application_test/logger_test.rs"]
mod tests;
