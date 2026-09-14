//! Unit tests for the Pub/Sub worker supervisor.

use std::{io, thread, time::Duration};

use crate::{
    devices::lidar::{LidarDecoder, LidarReader, RawPacket},
    models::pointcloud::{PointCloudFrame, PointXYZIRT},
};

use super::*;

struct MockReader;

impl LidarReader for MockReader {
    /// Simulates one hardware read that extends beyond the short test duration.
    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        thread::sleep(Duration::from_millis(2));
        Ok(RawPacket::new(vec![1, 2, 3]).with_timestamp_ns(100))
    }
}

struct MockDecoder;

impl LidarDecoder for MockDecoder {
    /// Converts the mock packet into one sensor-neutral point.
    fn decode_packet(&mut self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        assert_eq!(packet.as_bytes(), &[1, 2, 3]);
        Ok(PointCloudFrame::new(
            packet.timestamp_ns().unwrap_or_default(),
            vec![PointXYZIRT {
                x: 1.0,
                y: 0.0,
                z: 0.5,
                intensity: 1,
                ring: 0,
                return_id: 0,
                timestamp_ns: 100,
            }],
        ))
    }
}

/// Confirms independent Read, Decode, and Preprocessing workers exchange data.
#[test]
fn supervises_enabled_workers_without_creating_output_files() {
    let lidar = Lidar::from_parts(
        "Mock LiDAR",
        "mock-lidar",
        Box::new(MockReader),
        Box::new(MockDecoder),
    );
    let config = CaptureConfig {
        duration: Duration::from_millis(1),
        raw_path: None,
        pointcloud_path: None,
        preprocessing: PreprocessingConfig {
            min_distance_m: 0.0,
            max_distance_m: 10.0,
            region_of_interest: None,
            voxel_size_m: None,
            ground_removal: None,
        },
        threads: CaptureThreadConfig {
            lidar_read: ThreadConfig::new("test-lidar-read", 1).unwrap(),
            lidar_decode: ThreadConfig::new("test-lidar-decode", 2).unwrap(),
            raw_logger: ThreadConfig::new("test-raw-logger", 2).unwrap(),
            preprocessing: ThreadConfig::new("test-preprocessing", 3).unwrap(),
            pcd_logger: ThreadConfig::new("test-pcd-logger", 2).unwrap(),
        },
        queues: TopicQueueConfig {
            raw_capacity: 2,
            decoded_capacity: 2,
            processed_capacity: 2,
        },
    };

    let stats = run_capture(lidar, config).unwrap();
    assert_eq!(stats.packet_count, 1);
    assert_eq!(stats.point_count, 1);
}
