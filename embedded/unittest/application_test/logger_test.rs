//! Unit tests for RAW and PCD application logger workers.

use std::{
    fs,
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
};

use crate::{models::pointcloud::PointXYZIRT, platform::pubsub::Topic};

use super::*;

static NEXT_FILE_ID: AtomicU64 = AtomicU64::new(0);

/// Creates a unique path in the operating system's temporary directory.
fn temporary_output(extension: &str) -> PathBuf {
    let id = NEXT_FILE_ID.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "vista-lidar-logger-test-{}-{id}.{extension}",
        std::process::id()
    ))
}

/// Confirms that RAW logging preserves the exact sensor bytes.
#[test]
fn writes_raw_packets_without_modification() {
    let path = temporary_output("bin");
    let topic = Topic::new("test/lidar/raw", 2).unwrap();
    let subscriber = topic.subscribe("test-raw-logger").unwrap();
    let publisher = topic.publisher();

    publisher
        .publish(LidarMessage::new(
            "test-lidar",
            0,
            Some(10),
            20,
            RawPacket::new(vec![1, 2, 3, 4]),
        ))
        .unwrap();
    drop(publisher);

    let report = run_raw_logger(subscriber, path.clone()).unwrap();
    assert_eq!(report.message_count, 1);
    assert_eq!(fs::read(&path).unwrap(), vec![1, 2, 3, 4]);
    fs::remove_file(path).unwrap();
}

/// Confirms that PCD logging writes the final point count and point data.
#[test]
fn writes_processed_point_cloud_as_pcd() {
    let path = temporary_output("pcd");
    let topic = Topic::new("test/pointcloud/processed", 2).unwrap();
    let subscriber = topic.subscribe("test-pcd-logger").unwrap();
    let publisher = topic.publisher();
    let frame = PointCloudFrame::new(
        10,
        vec![PointXYZIRT {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 4,
            ring: 5,
            return_id: 0,
            timestamp_ns: 10,
        }],
    );

    publisher
        .publish(LidarMessage::new("test-lidar", 0, Some(10), 20, frame))
        .unwrap();
    drop(publisher);

    let report = run_pcd_logger(subscriber, path.clone()).unwrap();
    let contents = fs::read_to_string(&path).unwrap();
    assert_eq!(report.message_count, 1);
    assert_eq!(report.point_count, 1);
    assert!(contents.contains("POINTS 1"));
    assert!(contents.contains("1.000000 2.000000 3.000000 4 5 0"));
    fs::remove_file(path).unwrap();
}
