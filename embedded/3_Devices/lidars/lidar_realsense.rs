//! Intel RealSense L515 device driver and Z16 point-cloud decoder.
//!
//! USB discovery, native handles, and unsafe librealsense calls live in the
//! Transport layer. This driver only supplies L515 settings and interprets data.

use std::{io, io::ErrorKind};

use crate::{
    devices::lidar::{DepthStreamConfig, LidarDecoder, LidarReader, RawPacket},
    models::pointcloud::{PointCloudFrame, PointXYZIRT},
    transport::librealsense_usb::{
        LibrealsenseDepthFrame, LibrealsenseDepthStreamConfig, LibrealsenseUsbConnection,
    },
};

/// Namespace used by the common LiDAR factory to initialize both worker halves.
pub struct RealSenseL515;

/// Owns the safe USB transport and runs exclusively in the LiDAR Read thread.
pub struct RealSenseL515Reader {
    connection: LibrealsenseUsbConnection,
    pending_packet: Option<RawPacket>,
}

/// Owns copied calibration and runs independently in the LiDAR Decode thread.
pub struct RealSenseL515Decoder {
    depth_scale_m: f32,
    rays: Vec<[f32; 3]>,
}

impl RealSenseL515 {
    /// Opens the L515 through Transport and separates Read state from Decode state.
    pub fn connect(
        config: DepthStreamConfig,
    ) -> io::Result<(RealSenseL515Reader, RealSenseL515Decoder)> {
        let transport_config = LibrealsenseDepthStreamConfig::new(
            "L515",
            config.width,
            config.height,
            config.frames_per_second,
            config.frame_timeout,
        );
        let mut connection = LibrealsenseUsbConnection::open(transport_config)?;

        // The first frame supplies the calibration used by the independent decoder.
        // Keep that frame pending so sequence zero is not silently discarded.
        let first_frame = connection.read_depth_frame()?;
        let calibration = connection.lock_calibration()?;
        let pending_packet = Some(raw_packet_from_frame(first_frame));

        Ok((
            RealSenseL515Reader {
                connection,
                pending_packet,
            },
            RealSenseL515Decoder {
                depth_scale_m: calibration.depth_scale_m,
                rays: calibration.rays,
            },
        ))
    }
}

impl LidarReader for RealSenseL515Reader {
    /// Requests one Z16 frame from the USB transport and wraps it as LiDAR RAW data.
    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        if let Some(packet) = self.pending_packet.take() {
            return Ok(packet);
        }

        self.connection
            .read_depth_frame()
            .map(raw_packet_from_frame)
    }
}

impl LidarDecoder for RealSenseL515Decoder {
    /// Converts packed Z16 pixels into the shared X-forward/Y-left/Z-up point cloud.
    fn decode_packet(&mut self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        decode_depth_frame(
            packet.as_bytes(),
            &self.rays,
            self.depth_scale_m,
            packet.timestamp_ns().unwrap_or_default(),
        )
    }
}

/// Converts one transport-owned frame into the common LiDAR RAW packet model.
fn raw_packet_from_frame(frame: LibrealsenseDepthFrame) -> RawPacket {
    RawPacket::new(frame.bytes).with_timestamp_ns(frame.timestamp_ns)
}

/// Converts a packed Z16 image and cached optical rays into common coordinates.
fn decode_depth_frame(
    bytes: &[u8],
    rays: &[[f32; 3]],
    depth_scale_m: f32,
    timestamp_ns: u64,
) -> io::Result<PointCloudFrame> {
    let expected_size = rays
        .len()
        .checked_mul(2)
        .ok_or_else(|| io::Error::new(ErrorKind::InvalidData, "L515 depth buffer size overflow"))?;
    if bytes.len() != expected_size {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "L515 depth buffer contains {} bytes; expected {expected_size}",
                bytes.len()
            ),
        ));
    }

    let mut points = Vec::with_capacity(rays.len());
    for (pixel, ray) in bytes.chunks_exact(2).zip(rays) {
        let raw_depth = u16::from_le_bytes([pixel[0], pixel[1]]);
        if raw_depth == 0 {
            continue;
        }
        let depth_m = f32::from(raw_depth) * depth_scale_m;

        // RealSense optical coordinates are X-right, Y-down, Z-forward.
        // Convert them to the application convention: X-forward, Y-left, Z-up.
        points.push(PointXYZIRT {
            x: ray[2] * depth_m,
            y: -ray[0] * depth_m,
            z: -ray[1] * depth_m,
            intensity: 0,
            ring: 0,
            return_id: 0,
            timestamp_ns,
        });
    }

    Ok(PointCloudFrame::new(timestamp_ns, points))
}

#[cfg(test)]
#[path = "../../unittest/device_test/lidar_realsense_test.rs"]
mod tests;
