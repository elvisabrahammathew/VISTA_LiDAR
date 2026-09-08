//! Common types and behaviour shared by LiDAR drivers.

use std::io;

// Owns one complete packet exactly as it arrived from a sensor.
#[derive(Debug, Clone)]
pub struct RawPacket {
    bytes: Vec<u8>,
}

impl RawPacket {
    pub fn new(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

// New LiDAR drivers implement this trait to plug into the capture pipeline.
pub trait LidarDevice {
    fn read_raw_packet(&mut self) -> io::Result<RawPacket>;
}
