//! TCP driver, packet framing, and decoding for the Quanergy M8 LiDAR.

use std::{
    f64::consts::TAU,
    io::{self, ErrorKind, Read},
    net::{SocketAddr, TcpStream},
    time::Duration,
};

use crate::{
    devices::lidar::{LidarDevice, RawPacket},
    models::pointcloud::{PointCloudFrame, PointXYZIRT},
    platform,
};

// Packet constants from the Quanergy M8 network protocol.
pub const DEFAULT_PORT: u16 = 4141;
const PACKET_SIGNATURE: u32 = 0x75bd_7e97;
const PACKET_HEADER_SIZE: usize = 20;
const ALL_RETURNS_PACKET_TYPE: u8 = 0x00;
const REDUCED_RETURN_PACKET_TYPE: u8 = 0x04;
const ALL_RETURNS_PACKET_SIZE: usize = 6_632;
const REDUCED_RETURN_PACKET_SIZE: usize = 2_224;

// Packet dimensions and measurement units defined by the M8 manual.
const FIRINGS_PER_PACKET: usize = 50;
const LASER_COUNT: usize = 8;
const RETURN_COUNT: usize = 3;
const ALL_RETURNS_FIRING_SIZE: usize = 132;
const REDUCED_RETURN_FIRING_SIZE: usize = 44;
const POSITION_STEPS_PER_ROTATION: f64 = 10_400.0;
const DISTANCE_UNIT_METERS: f64 = 0.000_01;

// Vertical beam angles ordered from the lowest to the highest laser.
const VERTICAL_ANGLES: [f64; LASER_COUNT] = [
    -0.318_505,
    -0.269_2,
    -0.218_009,
    -0.165_195,
    -0.111_003,
    -0.055_798_2,
    0.0,
    0.055_798_2,
];

/// Owns the TCP connection used to receive packets from one Quanergy M8.
pub struct QuanergyM8 {
    stream: TcpStream,
}

impl QuanergyM8 {
    /// Opens the sensor TCP stream and applies platform-specific socket settings.
    pub fn connect(address: SocketAddr) -> io::Result<Self> {
        let stream = TcpStream::connect_timeout(&address, Duration::from_secs(5))?;
        platform::configure_sensor_stream(&stream)?;
        Ok(Self { stream })
    }

    /// Validates the fixed header and returns the complete packet size.
    fn validate_header(header: &[u8; PACKET_HEADER_SIZE]) -> io::Result<usize> {
        // M8 multi-byte fields use network byte order (Big Endian).
        let signature = u32::from_be_bytes(header[0..4].try_into().expect("fixed slice"));
        if signature != PACKET_SIGNATURE {
            return Err(invalid_data(format!(
                "invalid M8 packet signature 0x{signature:08x}"
            )));
        }

        let message_size =
            u32::from_be_bytes(header[4..8].try_into().expect("fixed slice")) as usize;
        let packet_type = header[19];
        let expected_size = match packet_type {
            ALL_RETURNS_PACKET_TYPE => ALL_RETURNS_PACKET_SIZE,
            REDUCED_RETURN_PACKET_TYPE => REDUCED_RETURN_PACKET_SIZE,
            other => {
                return Err(invalid_data(format!(
                    "unsupported M8 packet type 0x{other:02x}"
                )));
            }
        };

        if message_size != expected_size {
            return Err(invalid_data(format!(
                "M8 packet type 0x{packet_type:02x} declares {message_size} bytes; \
                 expected {expected_size}"
            )));
        }

        Ok(message_size)
    }

    /// Decodes either supported M8 packet format into a sensor-neutral frame.
    fn decode(&self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        let bytes = packet.as_bytes();
        let packet_type = validate_common_header(bytes)?;
        let packet_timestamp_ns = timestamp_ns(bytes)?;

        match packet_type {
            ALL_RETURNS_PACKET_TYPE => decode_all_returns(bytes, packet_timestamp_ns),
            REDUCED_RETURN_PACKET_TYPE => decode_reduced_return(bytes, packet_timestamp_ns),
            other => Err(invalid_data(format!(
                "unsupported M8 packet type 0x{other:02x}"
            ))),
        }
    }
}

impl LidarDevice for QuanergyM8 {
    /// Identifies this concrete driver through the common LiDAR interface.
    fn device_name(&self) -> &'static str {
        "Quanergy M8"
    }

    /// Reads one complete M8 packet even when TCP splits it across segments.
    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        // TCP is a byte stream, so first read the fixed header to learn packet size.
        let mut header = [0_u8; PACKET_HEADER_SIZE];
        self.stream.read_exact(&mut header)?;
        let message_size = Self::validate_header(&header)?;

        let mut bytes = vec![0_u8; message_size];
        bytes[..PACKET_HEADER_SIZE].copy_from_slice(&header);
        self.stream.read_exact(&mut bytes[PACKET_HEADER_SIZE..])?;

        Ok(RawPacket::new(bytes))
    }

    /// Delegates decoding to the Quanergy-specific packet decoder.
    fn decode_packet(&self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        self.decode(packet)
    }
}

/// Creates an InvalidData error with a consistent error kind.
fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, message.into())
}

/// Reads a Big-Endian 16-bit value and detects truncated packets.
fn read_u16_be(bytes: &[u8], offset: usize) -> io::Result<u16> {
    let value = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| invalid_data("truncated M8 uint16 field"))?;
    Ok(u16::from_be_bytes(
        value.try_into().expect("two-byte slice"),
    ))
}

/// Reads a Big-Endian 32-bit value and detects truncated packets.
fn read_u32_be(bytes: &[u8], offset: usize) -> io::Result<u32> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| invalid_data("truncated M8 uint32 field"))?;
    Ok(u32::from_be_bytes(
        value.try_into().expect("four-byte slice"),
    ))
}

/// Combines the packet seconds and nanoseconds fields into one nanosecond value.
fn timestamp_ns(bytes: &[u8]) -> io::Result<u64> {
    let seconds = u64::from(read_u32_be(bytes, 8)?);
    let nanoseconds = u64::from(read_u32_be(bytes, 12)?);
    if nanoseconds >= 1_000_000_000 {
        return Err(invalid_data(format!(
            "invalid packet timestamp nanoseconds: {nanoseconds}"
        )));
    }
    Ok(seconds * 1_000_000_000 + nanoseconds)
}

/// Adds one valid polar measurement as a Cartesian point.
fn push_point(
    points: &mut Vec<PointXYZIRT>,
    position: u16,
    laser: usize,
    return_id: u8,
    distance_raw: u32,
    intensity: u8,
    timestamp_ns: u64,
) {
    // A zero distance is the M8 marker for an invalid measurement.
    if distance_raw == 0 {
        return;
    }

    let azimuth = f64::from(position) * TAU / POSITION_STEPS_PER_ROTATION;
    let vertical = VERTICAL_ANGLES[laser];
    let distance = f64::from(distance_raw) * DISTANCE_UNIT_METERS;
    let horizontal_distance = distance * vertical.cos();

    points.push(PointXYZIRT {
        x: (horizontal_distance * azimuth.cos()) as f32,
        y: (horizontal_distance * azimuth.sin()) as f32,
        z: (distance * vertical.sin()) as f32,
        intensity,
        ring: laser as u8,
        return_id,
        timestamp_ns,
    });
}

/// Revalidates the complete packet header before decoding measurement fields.
fn validate_common_header(bytes: &[u8]) -> io::Result<u8> {
    if bytes.len() < PACKET_HEADER_SIZE {
        return Err(invalid_data("truncated M8 packet header"));
    }

    let signature = read_u32_be(bytes, 0)?;
    if signature != PACKET_SIGNATURE {
        return Err(invalid_data(format!(
            "invalid M8 packet signature 0x{signature:08x}"
        )));
    }

    let declared_size = read_u32_be(bytes, 4)? as usize;
    if declared_size != bytes.len() {
        return Err(invalid_data(format!(
            "M8 packet declares {declared_size} bytes but contains {}",
            bytes.len()
        )));
    }

    Ok(bytes[19])
}

/// Decodes an all-three-returns packet containing three returns for eight lasers.
fn decode_all_returns(bytes: &[u8], packet_timestamp_ns: u64) -> io::Result<PointCloudFrame> {
    if bytes.len() != ALL_RETURNS_PACKET_SIZE {
        return Err(invalid_data("incorrect all-returns packet size"));
    }

    // Status follows 50 firing blocks and the trailing timestamp/API fields.
    let status_offset = PACKET_HEADER_SIZE + FIRINGS_PER_PACKET * ALL_RETURNS_FIRING_SIZE + 10;
    let status = read_u16_be(bytes, status_offset)?;
    if status != 0 {
        return Err(invalid_data(format!("M8 packet status is 0x{status:04x}")));
    }

    let mut points = Vec::with_capacity(FIRINGS_PER_PACKET * LASER_COUNT * RETURN_COUNT);
    for firing in 0..FIRINGS_PER_PACKET {
        let base = PACKET_HEADER_SIZE + firing * ALL_RETURNS_FIRING_SIZE;
        let position = read_u16_be(bytes, base)?;
        if position >= 10_400 {
            return Err(invalid_data(format!("invalid firing position: {position}")));
        }

        let distances_offset = base + 4;
        let intensities_offset = distances_offset + RETURN_COUNT * LASER_COUNT * 4;
        for return_id in 0..RETURN_COUNT {
            for laser in 0..LASER_COUNT {
                let sample = return_id * LASER_COUNT + laser;
                let distance = read_u32_be(bytes, distances_offset + sample * 4)?;
                let intensity = bytes[intensities_offset + sample];
                push_point(
                    &mut points,
                    position,
                    laser,
                    return_id as u8,
                    distance,
                    intensity,
                    packet_timestamp_ns,
                );
            }
        }
    }

    Ok(PointCloudFrame::new(packet_timestamp_ns, points))
}

/// Decodes a reduced-return packet containing one selected return per laser.
fn decode_reduced_return(bytes: &[u8], packet_timestamp_ns: u64) -> io::Result<PointCloudFrame> {
    if bytes.len() != REDUCED_RETURN_PACKET_SIZE {
        return Err(invalid_data("incorrect reduced-return packet size"));
    }

    let status = read_u16_be(bytes, PACKET_HEADER_SIZE)?;
    if status != 0 {
        return Err(invalid_data(format!("M8 packet status is 0x{status:04x}")));
    }

    let return_id = bytes[PACKET_HEADER_SIZE + 2];
    if return_id >= RETURN_COUNT as u8 {
        return Err(invalid_data(format!("invalid return ID: {return_id}")));
    }

    let firing_data_offset = PACKET_HEADER_SIZE + 4;
    let mut points = Vec::with_capacity(FIRINGS_PER_PACKET * LASER_COUNT);
    for firing in 0..FIRINGS_PER_PACKET {
        let base = firing_data_offset + firing * REDUCED_RETURN_FIRING_SIZE;
        let position = read_u16_be(bytes, base)?;
        if position >= 10_400 {
            return Err(invalid_data(format!("invalid firing position: {position}")));
        }

        let distances_offset = base + 4;
        let intensities_offset = distances_offset + LASER_COUNT * 4;
        for laser in 0..LASER_COUNT {
            let distance = read_u32_be(bytes, distances_offset + laser * 4)?;
            let intensity = bytes[intensities_offset + laser];
            push_point(
                &mut points,
                position,
                laser,
                return_id,
                distance,
                intensity,
                packet_timestamp_ns,
            );
        }
    }

    Ok(PointCloudFrame::new(packet_timestamp_ns, points))
}

#[cfg(test)]
#[path = "../../unittest/device_test/lidar_quanergym8_test.rs"]
mod tests;
