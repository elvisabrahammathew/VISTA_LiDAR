//! Common LiDAR interface, configuration, and driver selection.

use std::{
    fmt,
    io::{self, ErrorKind},
    net::{IpAddr, SocketAddr},
    str::FromStr,
};

use crate::{
    devices::lidar_quanergym8::{QuanergyM8, DEFAULT_PORT as QUANERGY_M8_DEFAULT_PORT},
    models::pointcloud::PointCloudFrame,
};

/// Lists the LiDAR models that can be selected by application configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarType {
    QuanergyM8,
    Unitree4d,
}

impl fmt::Display for LidarType {
    /// Writes the stable configuration name used by the command line.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::QuanergyM8 => formatter.write_str("quanergy-m8"),
            Self::Unitree4d => formatter.write_str("unitree-4d"),
        }
    }
}

impl FromStr for LidarType {
    type Err = String;

    /// Converts a user-provided LiDAR name into a supported configuration value.
    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.to_ascii_lowercase().as_str() {
            "quanergy-m8" | "quanergy" | "m8" => Ok(Self::QuanergyM8),
            "unitree-4d" | "unitree4d" | "unitree" => Ok(Self::Unitree4d),
            _ => Err(format!(
                "unsupported LiDAR type '{value}'; supported values: quanergy-m8, unitree-4d"
            )),
        }
    }
}

/// Contains the values needed to create the selected LiDAR driver.
#[derive(Debug, Clone, Copy)]
pub struct LidarConfig {
    pub lidar_type: LidarType,
    pub sensor_ip: IpAddr,
    pub port: Option<u16>,
}

impl LidarConfig {
    /// Creates a LiDAR configuration that uses the driver's default TCP port.
    pub fn new(lidar_type: LidarType, sensor_ip: IpAddr) -> Self {
        Self {
            lidar_type,
            sensor_ip,
            port: None,
        }
    }

    /// Overrides the driver's default port when a deployment requires it.
    pub fn with_port(mut self, port: u16) -> Self {
        self.port = Some(port);
        self
    }
}

/// Owns one complete packet exactly as it arrived from a sensor.
#[derive(Debug, Clone)]
pub struct RawPacket {
    bytes: Vec<u8>,
}

impl RawPacket {
    /// Wraps a complete sensor packet without changing its bytes.
    pub fn new(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }

    /// Borrows the original packet bytes for decoding or file output.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Returns the packet length in bytes.
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    /// Reports whether the packet contains no bytes.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

/// Defines the sensor operations used by the application pipeline.
pub trait LidarDevice {
    /// Returns a human-readable name for logs and diagnostics.
    fn device_name(&self) -> &'static str;

    /// Reads one complete packet in the sensor's original wire format.
    fn read_raw_packet(&mut self) -> io::Result<RawPacket>;

    /// Converts one device-specific raw packet into the common point-cloud model.
    fn decode_packet(&self, packet: &RawPacket) -> io::Result<PointCloudFrame>;
}

/// Hides the selected concrete driver from the application layer.
pub struct Lidar {
    driver: Box<dyn LidarDevice>,
}

impl Lidar {
    /// Creates and connects the concrete driver selected in LidarConfig.
    pub fn connect(config: &LidarConfig) -> io::Result<Self> {
        let driver: Box<dyn LidarDevice> = match config.lidar_type {
            LidarType::QuanergyM8 => {
                let port = config.port.unwrap_or(QUANERGY_M8_DEFAULT_PORT);
                let address = SocketAddr::new(config.sensor_ip, port);
                Box::new(QuanergyM8::connect(address)?)
            }
            LidarType::Unitree4d => {
                return Err(io::Error::new(
                    ErrorKind::Unsupported,
                    "the Unitree 4D driver has not been implemented yet",
                ));
            }
        };

        Ok(Self { driver })
    }

    /// Returns the name reported by the selected concrete driver.
    pub fn device_name(&self) -> &'static str {
        self.driver.device_name()
    }

    /// Delegates raw packet reading to the selected concrete driver.
    pub fn read_raw_packet(&mut self) -> io::Result<RawPacket> {
        self.driver.read_raw_packet()
    }

    /// Delegates packet decoding to the selected concrete driver.
    pub fn decode_packet(&self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        self.driver.decode_packet(packet)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verifies the short and full Quanergy configuration names.
    #[test]
    fn parses_quanergy_aliases() {
        assert_eq!("quanergy-m8".parse(), Ok(LidarType::QuanergyM8));
        assert_eq!("m8".parse(), Ok(LidarType::QuanergyM8));
    }

    /// Verifies that unknown LiDAR names produce a useful configuration error.
    #[test]
    fn rejects_unknown_lidar_type() {
        assert!("unknown-lidar".parse::<LidarType>().is_err());
    }
}
