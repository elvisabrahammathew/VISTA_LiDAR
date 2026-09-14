//! Common LiDAR interface, configuration, and driver selection.

use std::{
    fmt,
    io::{self, ErrorKind},
    net::{IpAddr, SocketAddr},
    str::FromStr,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use crate::{
    devices::lidar_quanergym8::{QuanergyM8, DEFAULT_PORT as QUANERGY_M8_DEFAULT_PORT},
    devices::lidar_realsense::RealSenseL515,
    models::{lidar_message::LidarMessage, pointcloud::PointCloudFrame},
    platform::{
        pubsub::{TopicPublisher, TopicSubscriber},
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
    },
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

/// RAW LiDAR message published by every supported LiDAR reader.
pub type LidarRawMessage = LidarMessage<RawPacket>;

/// Sensor-neutral point-cloud message published by every LiDAR decoder.
pub type LidarPointCloudMessage = LidarMessage<PointCloudFrame>;

/// Counts messages handled by one LiDAR worker.
#[derive(Debug, Default)]
pub(crate) struct LidarWorkerReport {
    pub message_count: u64,
}

/// Reads complete sensor packets and exclusively owns the hardware connection.
pub trait LidarReader: Send {
    /// Reads one complete packet in the sensor's original wire format.
    fn read_raw_packet(&mut self) -> io::Result<RawPacket>;
}

/// Converts device-specific packets without owning the hardware connection.
pub trait LidarDecoder: Send {
    /// Converts one device-specific raw packet into the common point-cloud model.
    fn decode_packet(&mut self, packet: &RawPacket) -> io::Result<PointCloudFrame>;
}

/// Owns both halves only during startup, before they move to separate threads.
pub struct Lidar {
    device_name: &'static str,
    lidar_id: &'static str,
    reader: Box<dyn LidarReader>,
    decoder: Box<dyn LidarDecoder>,
}

/// Parts moved into the independent Read and Decode workers.
pub struct LidarParts {
    pub device_name: &'static str,
    pub lidar_id: &'static str,
    pub reader: Box<dyn LidarReader>,
    pub decoder: Box<dyn LidarDecoder>,
}

impl Lidar {
    /// Creates the concrete reader and decoder selected in `LidarConfig`.
    pub fn connect(config: &LidarConfig) -> io::Result<Self> {
        let (device_name, lidar_id, reader, decoder): (
            &'static str,
            &'static str,
            Box<dyn LidarReader>,
            Box<dyn LidarDecoder>,
        ) = match config.lidar_type {
            LidarType::QuanergyM8 => {
                let port = config.port.unwrap_or(QUANERGY_M8_DEFAULT_PORT);
                let address = SocketAddr::new(config.sensor_ip, port);
                let (reader, decoder) = QuanergyM8::connect(address)?;
                (
                    "Quanergy M8",
                    "quanergy-m8",
                    Box::new(reader),
                    Box::new(decoder),
                )
            }
            LidarType::RealSenseL515 => {
                let depth_stream = config.depth_stream.ok_or_else(|| {
                    io::Error::new(
                        ErrorKind::InvalidInput,
                        "a depth stream configuration is required for RealSense L515",
                    )
                })?;
                let (reader, decoder) = RealSenseL515::connect(depth_stream)?;
                (
                    "Intel RealSense L515",
                    "realsense-l515",
                    Box::new(reader),
                    Box::new(decoder),
                )
            }
            LidarType::Unitree4d => {
                return Err(io::Error::new(
                    ErrorKind::Unsupported,
                    "the Unitree 4D driver has not been implemented yet",
                ));
            }
        };

        Ok(Self {
            device_name,
            lidar_id,
            reader,
            decoder,
        })
    }

    /// Returns the name reported during driver creation.
    pub fn device_name(&self) -> &'static str {
        self.device_name
    }

    /// Separates hardware reading from decoding so each can own one OS thread.
    pub fn into_parts(self) -> LidarParts {
        LidarParts {
            device_name: self.device_name,
            lidar_id: self.lidar_id,
            reader: self.reader,
            decoder: self.decoder,
        }
    }

    /// Creates a sensor-independent LiDAR for supervisor unit tests.
    #[cfg(test)]
    pub(crate) fn from_parts(
        device_name: &'static str,
        lidar_id: &'static str,
        reader: Box<dyn LidarReader>,
        decoder: Box<dyn LidarDecoder>,
    ) -> Self {
        Self {
            device_name,
            lidar_id,
            reader,
            decoder,
        }
    }
}

/// Starts the device-owned worker that reads and publishes RAW packets.
pub(crate) fn spawn_lidar_read_worker<F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    mut reader: Box<dyn LidarReader>,
    publisher: TopicPublisher<LidarRawMessage>,
    lidar_id: String,
    duration: Duration,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LidarWorkerReport, String>) + Send + 'static,
{
    let worker_stop = stop.clone();
    spawn_lidar_worker(thread_config, stop, on_complete, move || {
        run_lidar_reader(reader.as_mut(), publisher, lidar_id, duration, worker_stop)
    })
}

/// Starts the device-owned worker that converts RAW packets to point clouds.
pub(crate) fn spawn_lidar_decode_worker<F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    mut decoder: Box<dyn LidarDecoder>,
    subscriber: TopicSubscriber<LidarRawMessage>,
    publisher: TopicPublisher<LidarPointCloudMessage>,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LidarWorkerReport, String>) + Send + 'static,
{
    spawn_lidar_worker(thread_config, stop, on_complete, move || {
        run_lidar_decoder(decoder.as_mut(), subscriber, publisher)
    })
}

/// Applies common shutdown and completion behavior around a LiDAR worker loop.
fn spawn_lidar_worker<W, F>(
    thread_config: ThreadConfig,
    stop: StopToken,
    on_complete: F,
    worker: W,
) -> io::Result<WorkerHandle<()>>
where
    W: FnOnce() -> io::Result<LidarWorkerReport> + Send + 'static,
    F: FnOnce(Result<LidarWorkerReport, String>) + Send + 'static,
{
    spawn_worker(thread_config, move || {
        let result = worker().map_err(|error| error.to_string());
        if result.is_err() {
            stop.request_stop();
        }
        on_complete(result);
    })
}

/// Reads sensor-native packets and publishes them without decoding or logging.
fn run_lidar_reader(
    reader: &mut dyn LidarReader,
    publisher: TopicPublisher<LidarRawMessage>,
    lidar_id: String,
    duration: Duration,
    stop: StopToken,
) -> io::Result<LidarWorkerReport> {
    let started = Instant::now();
    let mut report = LidarWorkerReport::default();

    while started.elapsed() < duration && !stop.is_stop_requested() {
        let packet = reader.read_raw_packet()?;
        let sensor_timestamp_ns = packet.timestamp_ns();
        let received_timestamp_ns = system_timestamp_ns();
        publisher.publish(LidarMessage::new(
            lidar_id.clone(),
            report.message_count,
            sensor_timestamp_ns,
            received_timestamp_ns,
            packet,
        ))?;
        report.message_count += 1;
    }

    Ok(report)
}

/// Receives RAW messages and invokes the selected device-specific decoder.
fn run_lidar_decoder(
    decoder: &mut dyn LidarDecoder,
    subscriber: TopicSubscriber<LidarRawMessage>,
    publisher: TopicPublisher<LidarPointCloudMessage>,
) -> io::Result<LidarWorkerReport> {
    let mut report = LidarWorkerReport::default();

    while let Ok(message) = subscriber.recv() {
        let frame = decoder.decode_packet(&message.payload)?;
        publisher.publish(LidarMessage::new(
            message.lidar_id.clone(),
            message.sequence,
            message.sensor_timestamp_ns,
            message.received_timestamp_ns,
            frame,
        ))?;
        report.message_count += 1;
    }

    Ok(report)
}

/// Records host wall-clock time immediately after a packet read completes.
fn system_timestamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
        .min(u128::from(u64::MAX)) as u64
}

#[cfg(test)]
#[path = "../../unittest/device_test/lidar_test.rs"]
mod tests;
