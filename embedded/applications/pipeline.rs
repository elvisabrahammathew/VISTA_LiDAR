//! Coordinates one timed capture from sensor input to both output files.

use std::{
    io,
    net::SocketAddr,
    path::Path,
    time::{Duration, Instant},
};

use crate::{
    application::pointcloud_processing::decode_quanergy_m8,
    connection::local::{PcdWriter, RawCaptureWriter},
    devices::{lidar::LidarDevice, lidar_quanergym8::QuanergyM8},
};

#[derive(Debug, Clone, Copy)]
pub struct CaptureStats {
    pub packet_count: u64,
    pub point_count: u64,
    pub elapsed: Duration,
}

pub fn capture_quanergy_m8(
    address: SocketAddr,
    duration: Duration,
    raw_path: &Path,
    pointcloud_path: &Path,
) -> io::Result<CaptureStats> {
    let mut sensor = QuanergyM8::connect(address)?;
    let mut raw_writer = RawCaptureWriter::create(raw_path)?;
    let mut pcd_writer = PcdWriter::create(pointcloud_path)?;

    let started = Instant::now();
    let mut packet_count = 0_u64;
    let mut point_count = 0_u64;

    while started.elapsed() < duration {
        // Keep the original packet before performing any conversion.
        let packet = sensor.read_raw_packet()?;
        raw_writer.write_packet(&packet)?;

        // Decode the same packet and stream its points to the PCD writer.
        let frame = decode_quanergy_m8(&packet)?;
        point_count += frame.points.len() as u64;
        pcd_writer.write_frame(&frame)?;
        packet_count += 1;
    }

    // Finalizing the PCD writer adds its header with the final point count.
    raw_writer.finish()?;
    let written_point_count = pcd_writer.finish()?;
    debug_assert_eq!(written_point_count, point_count);

    Ok(CaptureStats {
        packet_count,
        point_count,
        elapsed: started.elapsed(),
    })
}
