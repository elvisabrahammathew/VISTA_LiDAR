//! Common LiDAR interface, configuration, and driver selection.

use std::{
    fmt,
    io::{self, ErrorKind},
    net::{IpAddr, SocketAddr},
    str::FromStr,
    time::Duration,
};

use crate::{
    devices::lidar_quanergym8::{QuanergyM8, DEFAULT_PORT as QUANERGY_M8_DEFAULT_PORT},
    devices::lidar_realsense::RealSenseL515,
    models::pointcloud::PointCloudFrame,
};

/// Lists the LiDAR models that can be selected by application configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarType {
    QuanergyM8,
    RealSenseL515,
    Unitree4d,
}

impl fmt::Display for LidarType {
    /// Writes the stable configuration name used by the command line.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::QuanergyM8 => formatter.write_str("quanergy-m8"),
            Self::RealSenseL515 => formatter.write_str("realsense-l515"),
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
            "realsense-l515" | "realsense" | "l515" => Ok(Self::RealSenseL515),
            "unitree-4d" | "unitree4d" | "unitree" => Ok(Self::Unitree4d),
            _ => Err(format!(
                "unsupported LiDAR type '{value}'; supported values: \
                 quanergy-m8, realsense-l515, unitree-4d"
            )),
        }
    }
}

/// Selects the depth stream requested from a depth-camera LiDAR.
#[derive(Debug, Clone, Copy)]
pub struct DepthStreamConfig {
    pub width: u32,
    pub height: u32,
    pub frames_per_second: u32,
    pub frame_timeout: Duration,
}

impl DepthStreamConfig {
    /// Creates and validates a depth stream configuration at driver startup.
    pub fn new(width: u32, height: u32, frames_per_second: u32, frame_timeout: Duration) -> Self {
        Self {
            width,
            height,
            frames_per_second,
            frame_timeout,
        }
    }
}

/// Contains the values needed to create the selected LiDAR driver.
#[derive(Debug, Clone, Copy)]
pub struct LidarConfig {
    pub lidar_type: LidarType,
    pub sensor_ip: IpAddr,
    pub port: Option<u16>,
    pub depth_stream: Option<DepthStreamConfig>,
}

impl LidarConfig {
    /// Creates a LiDAR configuration that uses the driver's default TCP port.
    pub fn new(lidar_type: LidarType, sensor_ip: IpAddr) -> Self {
        Self {
            lidar_type,
            sensor_ip,
            port: None,
            depth_stream: None,
        }
    }

    /// Overrides the driver's default port when a deployment requires it.
    pub fn with_port(mut self, port: u16) -> Self {
        self.port = Some(port);
        self
    }

    /// Supplies the depth stream requested from a RealSense LiDAR.
    pub fn with_depth_stream(mut self, depth_stream: DepthStreamConfig) -> Self {
        self.depth_stream = Some(depth_stream);
        self
    }
}

/// Owns one complete packet exactly as it arrived from a sensor.
#[derive(Debug, Clone)]
pub struct RawPacket {
    bytes: Vec<u8>,
    timestamp_ns: Option<u64>,
}

impl RawPacket {
    /// Wraps a complete sensor packet without changing its bytes.
    pub fn new(bytes: Vec<u8>) -> Self {
        Self {
            bytes,
            timestamp_ns: None,
        }
    }

    /// Adds a source timestamp when it is not encoded inside the raw bytes.
    pub fn with_timestamp_ns(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = Some(timestamp_ns);
        self
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

    /// Returns an out-of-band source timestamp when the driver supplied one.
    pub fn timestamp_ns(&self) -> Option<u64> {
        self.timestamp_ns
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
            LidarType::RealSenseL515 => {
                let depth_stream = config.depth_stream.ok_or_else(|| {
                    io::Error::new(
                        ErrorKind::InvalidInput,
                        "a depth stream configuration is required for RealSense L515",
                    )
                })?;
                Box::new(RealSenseL515::connect(depth_stream)?)
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
#[path = "../../unittest/device_test/lidar_test.rs"]
mod tests;
