//! Unit tests for lidar_quanergym8.rs packet framing and decoding.

use super::*;

/// Builds a minimal reduced-return packet containing one valid point.
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

/// Verifies a reduced-return header with the documented packet size.
#[test]
fn validates_reduced_return_header() {
    let mut header = [0_u8; PACKET_HEADER_SIZE];
    header[0..4].copy_from_slice(&PACKET_SIGNATURE.to_be_bytes());
    header[4..8].copy_from_slice(&(REDUCED_RETURN_PACKET_SIZE as u32).to_be_bytes());
    header[19] = REDUCED_RETURN_PACKET_TYPE;

    assert_eq!(
        QuanergyM8::validate_header(&header).unwrap(),
        REDUCED_RETURN_PACKET_SIZE
    );
}

/// Verifies that unknown M8 packet types are rejected before body reading.
#[test]
fn rejects_unknown_packet_type() {
    let mut header = [0_u8; PACKET_HEADER_SIZE];
    header[0..4].copy_from_slice(&PACKET_SIGNATURE.to_be_bytes());
    header[4..8].copy_from_slice(&100_u32.to_be_bytes());
    header[19] = 0xff;

    assert!(QuanergyM8::validate_header(&header).is_err());
}

/// Verifies the polar-to-Cartesian conversion along the positive X axis.
#[test]
fn converts_one_meter_at_zero_degrees_to_positive_x() {
    let packet = reduced_packet(0, 6, 100_000);
    let frame = decode_reduced_return(packet.as_bytes(), 1_000_000_002).unwrap();
    assert_eq!(frame.points.len(), 1);

    let point = frame.points[0];
    assert!((point.x - 1.0).abs() < 1.0e-6);
    assert!(point.y.abs() < 1.0e-6);
    assert!(point.z.abs() < 1.0e-6);
    assert_eq!(point.intensity, 42);
    assert_eq!(point.ring, 6);
    assert_eq!(point.return_id, 2);
}

/// Verifies that position step 2600 maps to the positive Y axis.
#[test]
fn position_2600_points_toward_positive_y() {
    let packet = reduced_packet(2_600, 6, 100_000);
    let frame = decode_reduced_return(packet.as_bytes(), 1_000_000_002).unwrap();
    let point = frame.points[0];
    assert!(point.x.abs() < 1.0e-6);
    assert!((point.y - 1.0).abs() < 1.0e-6);
}
