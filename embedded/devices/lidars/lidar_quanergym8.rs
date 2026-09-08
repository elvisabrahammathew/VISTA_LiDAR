use std::{
    io::{self, ErrorKind, Read},
    net::{SocketAddr, TcpStream},
    time::Duration,
};

use crate::{
    devices::lidar::{LidarDevice, RawPacket},
    platform,
};

pub const DEFAULT_PORT: u16 = 4141;
pub const PACKET_SIGNATURE: u32 = 0x75bd_7e97;
pub const PACKET_HEADER_SIZE: usize = 20;
pub const ALL_RETURNS_PACKET_TYPE: u8 = 0x00;
pub const REDUCED_RETURN_PACKET_TYPE: u8 = 0x04;
pub const ALL_RETURNS_PACKET_SIZE: usize = 6_632;
pub const REDUCED_RETURN_PACKET_SIZE: usize = 2_224;

pub struct QuanergyM8 {
    stream: TcpStream,
}

impl QuanergyM8 {
    pub fn connect(address: SocketAddr) -> io::Result<Self> {
        let stream = TcpStream::connect_timeout(&address, Duration::from_secs(5))?;
        platform::configure_sensor_stream(&stream)?;
        Ok(Self { stream })
    }

    fn validate_header(header: &[u8; PACKET_HEADER_SIZE]) -> io::Result<usize> {
        let signature = u32::from_be_bytes(header[0..4].try_into().expect("fixed slice"));
        if signature != PACKET_SIGNATURE {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("invalid M8 packet signature 0x{signature:08x}"),
            ));
        }

        let message_size =
            u32::from_be_bytes(header[4..8].try_into().expect("fixed slice")) as usize;
        let packet_type = header[19];
        let expected_size = match packet_type {
            ALL_RETURNS_PACKET_TYPE => ALL_RETURNS_PACKET_SIZE,
            REDUCED_RETURN_PACKET_TYPE => REDUCED_RETURN_PACKET_SIZE,
            other => {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    format!("unsupported M8 packet type 0x{other:02x}"),
                ));
            }
        };

        if message_size != expected_size {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "M8 packet type 0x{packet_type:02x} declares {message_size} bytes; \
                     expected {expected_size}"
                ),
            ));
        }

        Ok(message_size)
    }
}

impl LidarDevice for QuanergyM8 {
    fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        let mut header = [0_u8; PACKET_HEADER_SIZE];
        self.stream.read_exact(&mut header)?;
        let message_size = Self::validate_header(&header)?;

        let mut bytes = vec![0_u8; message_size];
        bytes[..PACKET_HEADER_SIZE].copy_from_slice(&header);
        self.stream.read_exact(&mut bytes[PACKET_HEADER_SIZE..])?;

        Ok(RawPacket::new(bytes))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn rejects_unknown_packet_type() {
        let mut header = [0_u8; PACKET_HEADER_SIZE];
        header[0..4].copy_from_slice(&PACKET_SIGNATURE.to_be_bytes());
        header[4..8].copy_from_slice(&100_u32.to_be_bytes());
        header[19] = 0xff;

        assert!(QuanergyM8::validate_header(&header).is_err());
    }
}
