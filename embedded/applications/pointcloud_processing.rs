//! Decodes Quanergy M8 packets and converts polar measurements to XYZ points.

use std::{
    f64::consts::TAU,
    io::{self, ErrorKind},
};

use crate::{
    devices::{
        lidar::RawPacket,
        lidar_quanergym8::{
            ALL_RETURNS_PACKET_SIZE, ALL_RETURNS_PACKET_TYPE, PACKET_HEADER_SIZE, PACKET_SIGNATURE,
            REDUCED_RETURN_PACKET_SIZE, REDUCED_RETURN_PACKET_TYPE,
        },
    },
    models::pointcloud::{PointCloudFrame, PointXYZIRT},
};

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

fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, message.into())
}

fn read_u16_be(bytes: &[u8], offset: usize) -> io::Result<u16> {
    let value = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| invalid_data("truncated M8 uint16 field"))?;
    Ok(u16::from_be_bytes(
        value.try_into().expect("two-byte slice"),
    ))
}

fn read_u32_be(bytes: &[u8], offset: usize) -> io::Result<u32> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| invalid_data("truncated M8 uint32 field"))?;
    Ok(u32::from_be_bytes(
        value.try_into().expect("four-byte slice"),
    ))
}

fn timestamp_ns(bytes: &[u8]) -> io::Result<u64> {
    // The packet header stores seconds and nanoseconds as Big Endian values.
    let seconds = u64::from(read_u32_be(bytes, 8)?);
    let nanoseconds = u64::from(read_u32_be(bytes, 12)?);
    if nanoseconds >= 1_000_000_000 {
        return Err(invalid_data(format!(
            "invalid packet timestamp nanoseconds: {nanoseconds}"
        )));
    }
    Ok(seconds * 1_000_000_000 + nanoseconds)
}

fn push_point(
    points: &mut Vec<PointXYZIRT>,
    position: u16,
    laser: usize,
    return_id: u8,
    distance_raw: u32,
    intensity: u8,
    timestamp_ns: u64,
) {
    if distance_raw == 0 {
        return;
    }

    // Convert the M8 rotation step and beam angle from polar to Cartesian space.
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

    // Reduced packets contain one selected return for every laser.
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

pub fn decode_quanergy_m8(packet: &RawPacket) -> io::Result<PointCloudFrame> {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn reduced_packet(position: u16, laser: usize, distance: u32) -> RawPacket {
        let mut bytes = vec![0_u8; REDUCED_RETURN_PACKET_SIZE];
        bytes[0..4].copy_from_slice(&PACKET_SIGNATURE.to_be_bytes());
        bytes[4..8].copy_from_slice(&(REDUCED_RETURN_PACKET_SIZE as u32).to_be_bytes());
        bytes[8..12].copy_from_slice(&1_u32.to_be_bytes());
        bytes[12..16].copy_from_slice(&2_u32.to_be_bytes());
        bytes[16..19].copy_from_slice(&[0, 1, 0]);
        bytes[19] = REDUCED_RETURN_PACKET_TYPE;
        bytes[22] = 2;

        let firing_base = PACKET_HEADER_SIZE + 4;
        bytes[firing_base..firing_base + 2].copy_from_slice(&position.to_be_bytes());
        let distance_offset = firing_base + 4 + laser * 4;
        bytes[distance_offset..distance_offset + 4].copy_from_slice(&distance.to_be_bytes());
        bytes[firing_base + 36 + laser] = 42;
        RawPacket::new(bytes)
    }

    #[test]
    fn converts_one_meter_at_zero_degrees_to_positive_x() {
        let packet = reduced_packet(0, 6, 100_000);
        let frame = decode_quanergy_m8(&packet).unwrap();
        assert_eq!(frame.timestamp_ns, 1_000_000_002);
        assert_eq!(frame.points.len(), 1);

        let point = frame.points[0];
        assert!((point.x - 1.0).abs() < 1.0e-6);
        assert!(point.y.abs() < 1.0e-6);
        assert!(point.z.abs() < 1.0e-6);
        assert_eq!(point.intensity, 42);
        assert_eq!(point.ring, 6);
        assert_eq!(point.return_id, 2);
    }

    #[test]
    fn position_2600_points_toward_positive_y() {
        let packet = reduced_packet(2_600, 6, 100_000);
        let frame = decode_quanergy_m8(&packet).unwrap();
        let point = frame.points[0];
        assert!(point.x.abs() < 1.0e-6);
        assert!((point.y - 1.0).abs() < 1.0e-6);
    }
}
