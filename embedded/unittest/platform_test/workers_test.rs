//! Integration tests for independent workers and their self-registered topics.

use std::{
    io,
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc, Arc,
    },
    thread,
    time::Duration,
};

use crate::{
    application::pointcloud_processing::{
        spawn_preprocessing_worker, PreprocessingConfig, PreprocessingReport,
    },
    devices::lidar::{
        spawn_lidar_decode_worker, spawn_lidar_read_worker, Lidar, LidarConnector, LidarDecoder,
        LidarDecoderSlot, LidarReader, LidarWorkerReport, RawPacket,
    },
    models::{
        pointcloud::{PointCloudFrame, PointXYZIRT},
        topics,
    },
    platform::{
        message_bus::MessageBus,
        threading::{StopToken, ThreadConfig},
    },
};

struct MockReader {
    stop: StopToken,
}

impl LidarReader for MockReader {
    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        thread::sleep(Duration::from_millis(2));
        self.stop.request_stop();
        Ok(RawPacket::new(vec![1, 2, 3]).with_timestamp_ns(100))
    }
}

struct MockDecoder;

impl LidarDecoder for MockDecoder {
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

/// Consumers may start first because endpoint registration belongs to workers.
#[test]
fn workers_register_topics_and_process_one_packet() {
    let stop = StopToken::new();
    let bus = Arc::new(MessageBus::new(16).unwrap());
    bus.configure_topic(topics::LIDAR_RAW, 2).unwrap();
    bus.configure_topic(topics::POINTCLOUD_DECODED, 2).unwrap();
    bus.configure_topic(topics::POINTCLOUD_PROCESSED, 2)
        .unwrap();
    let decoder_slot = Arc::new(LidarDecoderSlot::new());
    let (preprocess_tx, preprocess_rx) = mpsc::channel();
    let (decode_tx, decode_rx) = mpsc::channel();
    let (read_tx, read_rx) = mpsc::channel();

    let preprocessing = spawn_preprocessing_worker(
        Arc::clone(&bus),
        ThreadConfig::new("test-preprocessing", 3).unwrap(),
        stop.clone(),
        PreprocessingConfig {
            min_distance_m: 0.0,
            max_distance_m: 10.0,
            region_of_interest: None,
            voxel_size_m: None,
            ground_removal: None,
        },
        move |result: Result<PreprocessingReport, String>| preprocess_tx.send(result).unwrap(),
    )
    .unwrap();
    let decoder = spawn_lidar_decode_worker(
        Arc::clone(&bus),
        ThreadConfig::new("test-lidar-decode", 2).unwrap(),
        stop.clone(),
        Arc::clone(&decoder_slot),
        move |result: Result<LidarWorkerReport, String>| decode_tx.send(result).unwrap(),
    )
    .unwrap();
    let connector_stop = stop.clone();
    let connector: LidarConnector = Arc::new(move || {
        Ok(Lidar::from_parts(
            "Mock LiDAR",
            "mock-lidar",
            Box::new(MockReader {
                stop: connector_stop.clone(),
            }),
            Box::new(MockDecoder),
        ))
    });
    let reader = spawn_lidar_read_worker(
        Arc::clone(&bus),
        ThreadConfig::new("test-lidar-read", 1).unwrap(),
        stop,
        connector,
        Duration::from_millis(1),
        decoder_slot,
        move |result: Result<LidarWorkerReport, String>| read_tx.send(result).unwrap(),
    )
    .unwrap();

    reader.join().unwrap();
    decoder.join().unwrap();
    preprocessing.join().unwrap();
    let read = read_rx.recv().unwrap().unwrap();
    let decode = decode_rx.recv().unwrap().unwrap();
    let preprocessing = preprocess_rx.recv().unwrap().unwrap();
    assert_eq!(read.message_count, 1);
    assert_eq!(decode.message_count, 1);
    assert_eq!(preprocessing.message_count, 1);
    assert_eq!(preprocessing.point_count, 1);
}

/// A failed connection is reported and retried instead of stopping the process.
#[test]
fn lidar_read_worker_retries_after_connection_failure() {
    let stop = StopToken::new();
    let bus = Arc::new(MessageBus::new(16).unwrap());
    bus.configure_topic(topics::LIDAR_RAW, 2).unwrap();
    let decoder_slot = Arc::new(LidarDecoderSlot::new());
    let attempts = Arc::new(AtomicUsize::new(0));
    let connector_attempts = Arc::clone(&attempts);
    let connector_stop = stop.clone();
    let connector: LidarConnector = Arc::new(move || {
        if connector_attempts.fetch_add(1, Ordering::SeqCst) == 0 {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "mock LiDAR is disconnected",
            ));
        }
        Ok(Lidar::from_parts(
            "Mock LiDAR",
            "mock-lidar",
            Box::new(MockReader {
                stop: connector_stop.clone(),
            }),
            Box::new(MockDecoder),
        ))
    });
    let (result_tx, result_rx) = mpsc::channel();
    let reader = spawn_lidar_read_worker(
        Arc::clone(&bus),
        ThreadConfig::new("test-lidar-reconnect", 1).unwrap(),
        stop,
        connector,
        Duration::from_millis(1),
        decoder_slot,
        move |result: Result<LidarWorkerReport, String>| result_tx.send(result).unwrap(),
    )
    .unwrap();

    reader.join().unwrap();
    let report = result_rx.recv().unwrap().unwrap();
    assert_eq!(attempts.load(Ordering::SeqCst), 2);
    assert_eq!(report.message_count, 1);
}
