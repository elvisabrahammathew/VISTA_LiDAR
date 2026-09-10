//! Coordinates one timed capture from a generic LiDAR to both output files.

use std::{
    io,
    path::Path,
    time::{Duration, Instant},
};

use crate::{
    application::pointcloud_processing::{preprocess_point_cloud, PreprocessingConfig},
    devices::lidar::Lidar,
    transport::local::{PcdWriter, RawCaptureWriter},
};

/// Summarizes the amount of data produced by one capture session.
#[derive(Debug, Clone, Copy)]
pub struct CaptureStats {
    pub packet_count: u64,
    pub point_count: u64,
    pub elapsed: Duration,
}

/// Captures, records, decodes, preprocesses, and writes data for a fixed duration.
pub fn capture_lidar(
    lidar: &mut Lidar,
    duration: Duration,
    raw_path: Option<&Path>,
    pointcloud_path: Option<&Path>,
    preprocessing: &PreprocessingConfig,
) -> io::Result<CaptureStats> {
    // Writers are created only for output paths explicitly enabled by main.
    let mut raw_writer = raw_path.map(RawCaptureWriter::create).transpose()?;
    let mut pcd_writer = pointcloud_path.map(PcdWriter::create).transpose()?;

    let started = Instant::now();
    let mut packet_count = 0_u64;
    let mut point_count = 0_u64;

    while started.elapsed() < duration {
        // Preserve every packet before any device-specific decoding or filtering.
        let packet = lidar.read_raw_packet()?;
        if let Some(writer) = raw_writer.as_mut() {
            writer.write_packet(&packet)?;
        }

        // The selected driver performs device-specific decoding behind the common interface.
        let decoded_frame = lidar.decode_packet(&packet)?;

        // Application processing only sees the sensor-neutral PointCloudFrame.
        let processed_frame = preprocess_point_cloud(decoded_frame, preprocessing)?;
        point_count += processed_frame.points.len() as u64;
        if let Some(writer) = pcd_writer.as_mut() {
            writer.write_frame(&processed_frame)?;
        }
        packet_count += 1;
    }

    // Flush and finalize only the outputs that were enabled.
    if let Some(writer) = raw_writer {
        writer.finish()?;
    }
    if let Some(writer) = pcd_writer {
        let written_point_count = writer.finish()?;
        debug_assert_eq!(written_point_count, point_count);
    }

    Ok(CaptureStats {
        packet_count,
        point_count,
        elapsed: started.elapsed(),
    })
}
