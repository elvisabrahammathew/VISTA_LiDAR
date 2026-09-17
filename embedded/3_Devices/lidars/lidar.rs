//! Common LiDAR facade, driver selection, and independent Read/Decode workers.

use std::{
    fmt,
    io::{self, ErrorKind},
    net::{IpAddr, SocketAddr},
    str::FromStr,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use crate::{
    devices::quanergym8::{QuanergyM8, DEFAULT_PORT as QUANERGY_M8_DEFAULT_PORT},
    devices::realsensel515::RealSenseL515,
    models::{lidar_message::LidarMessage, pointcloud::PointCloudFrame, topics},
    platform::{
        message_bus::{MessageBus, WorkerTopicInputs},
        pubsub::{ReceiveStatus, TopicPublisher, TopicSubscriber},
        threading::{spawn_worker, StopToken, ThreadConfig, WorkerHandle},
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LidarType {
    QuanergyM8,
    RealSenseL515,
    Unitree4d,
}

impl fmt::Display for LidarType {
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

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.to_ascii_lowercase().as_str() {
            "quanergy-m8" | "quanergym8" | "quanergy" | "m8" => Ok(Self::QuanergyM8),
            "realsense-l515" | "realsensel515" | "realsense" | "l515" => {
                Ok(Self::RealSenseL515)
            }
            "unitree-4d" | "unitree4d" | "unitree" => Ok(Self::Unitree4d),
            _ => Err(format!(
                "unsupported LiDAR type '{value}'; supported values: quanergy-m8, realsense-l515, unitree-4d"
            )),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DepthStreamConfig {
    pub usb_serial: Option<String>,
    pub width: u32,
    pub height: u32,
    pub frames_per_second: u32,
    pub frame_timeout: Duration,
}

impl DepthStreamConfig {
    pub fn new(
        usb_serial: Option<String>,
        width: u32,
        height: u32,
        frames_per_second: u32,
        frame_timeout: Duration,
    ) -> Self {
        Self {
            usb_serial,
            width,
            height,
            frames_per_second,
            frame_timeout,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LidarConfig {
    pub lidar_type: LidarType,
    pub sensor_ip: IpAddr,
    pub port: Option<u16>,
    pub depth_stream: Option<DepthStreamConfig>,
    pub connection_timeout: Duration,
}

impl LidarConfig {
    pub fn new(lidar_type: LidarType, sensor_ip: IpAddr) -> Self {
        Self {
            lidar_type,
            sensor_ip,
            port: None,
            depth_stream: None,
            connection_timeout: Duration::from_secs(5),
        }
    }

    pub fn with_port(mut self, port: u16) -> Self {
        self.port = Some(port);
        self
    }

    pub fn with_depth_stream(mut self, depth_stream: DepthStreamConfig) -> Self {
        self.depth_stream = Some(depth_stream);
        self
    }

    pub fn with_connection_timeout(mut self, timeout: Duration) -> Self {
        self.connection_timeout = timeout;
        self
    }
}

/// One complete packet exactly as it arrived from a sensor.
#[derive(Debug, Clone)]
pub struct RawPacket {
    bytes: Vec<u8>,
    timestamp_ns: Option<u64>,
}

impl RawPacket {
    pub fn new(bytes: Vec<u8>) -> Self {
        Self {
            bytes,
            timestamp_ns: None,
        }
    }

    pub fn with_timestamp_ns(mut self, timestamp_ns: u64) -> Self {
        self.timestamp_ns = Some(timestamp_ns);
        self
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

    pub fn timestamp_ns(&self) -> Option<u64> {
        self.timestamp_ns
    }
}

pub type LidarRawMessage = LidarMessage<RawPacket>;
pub type LidarPointCloudMessage = LidarMessage<PointCloudFrame>;

#[derive(Debug, Default)]
pub struct LidarWorkerReport {
    pub message_count: u64,
    pub dropped_message_count: u64,
}

pub trait LidarReader: Send {
    fn read_raw_packet(&mut self) -> io::Result<RawPacket>;
}

pub trait LidarDecoder: Send {
    fn decode_packet(&mut self, packet: &RawPacket) -> io::Result<PointCloudFrame>;
}

pub struct Lidar {
    device_name: &'static str,
    lidar_id: &'static str,
    reader: Box<dyn LidarReader>,
    decoder: Box<dyn LidarDecoder>,
}

pub struct LidarParts {
    pub device_name: &'static str,
    pub lidar_id: &'static str,
    pub reader: Box<dyn LidarReader>,
    pub decoder: Box<dyn LidarDecoder>,
}

impl Lidar {
    /// Creates a fresh device session for an initial connection or reconnect.
    pub fn connect(config: &LidarConfig) -> io::Result<Self> {
        let (device_name, lidar_id, reader, decoder): (
            &'static str,
            &'static str,
            Box<dyn LidarReader>,
            Box<dyn LidarDecoder>,
        ) = match config.lidar_type {
            LidarType::QuanergyM8 => {
                let address = SocketAddr::new(
                    config.sensor_ip,
                    config.port.unwrap_or(QUANERGY_M8_DEFAULT_PORT),
                );
                let (reader, decoder) = QuanergyM8::connect(address, config.connection_timeout)?;
                (
                    "Quanergy M8",
                    "quanergy-m8",
                    Box::new(reader),
                    Box::new(decoder),
                )
            }
            LidarType::RealSenseL515 => {
                let depth_stream = config.depth_stream.clone().ok_or_else(|| {
                    io::Error::new(
                        ErrorKind::InvalidInput,
                        "RealSense L515 requires a depth-stream configuration",
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
                    "the Unitree 4D driver is not implemented",
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

    pub fn device_name(&self) -> &'static str {
        self.device_name
    }

    pub fn into_parts(self) -> LidarParts {
        LidarParts {
            device_name: self.device_name,
            lidar_id: self.lidar_id,
            reader: self.reader,
            decoder: self.decoder,
        }
    }

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

/// Decoder shared between the read worker that reconnects and the decode worker.
#[derive(Default)]
pub struct LidarDecoderSlot {
    decoder: Mutex<Option<Box<dyn LidarDecoder>>>,
}

impl LidarDecoderSlot {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn replace(&self, decoder: Box<dyn LidarDecoder>) {
        *self
            .decoder
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = Some(decoder);
    }

    pub fn decode_packet(&self, packet: &RawPacket) -> io::Result<PointCloudFrame> {
        let mut decoder = self
            .decoder
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        decoder
            .as_mut()
            .ok_or_else(|| io::Error::new(ErrorKind::NotConnected, "LiDAR decoder is unavailable"))?
            .decode_packet(packet)
    }
}

pub type LidarConnector = Arc<dyn Fn() -> io::Result<Lidar> + Send + Sync>;

/// Starts the read worker; this worker owns connection and reconnection.
pub fn spawn_lidar_read_worker<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    connect_lidar: LidarConnector,
    reconnect_interval: Duration,
    decoder_slot: Arc<LidarDecoderSlot>,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LidarWorkerReport, String>) + Send + 'static,
{
    if reconnect_interval.is_zero() {
        return Err(io::Error::new(
            ErrorKind::InvalidInput,
            "LiDAR reconnect interval must be greater than zero",
        ));
    }
    let publisher = bus.publisher::<LidarRawMessage>(topics::LIDAR_RAW)?;
    spawn_worker(thread_config, move || {
        let result = run_lidar_reader(
            connect_lidar,
            publisher,
            reconnect_interval,
            decoder_slot,
            stop,
        )
        .map_err(|error| error.to_string());
        on_complete(result);
    })
}

/// Starts the decoder worker and lets it register its own topic endpoints.
pub fn spawn_lidar_decode_worker<F>(
    bus: Arc<MessageBus>,
    thread_config: ThreadConfig,
    stop: StopToken,
    decoder_slot: Arc<LidarDecoderSlot>,
    on_complete: F,
) -> io::Result<WorkerHandle<()>>
where
    F: FnOnce(Result<LidarWorkerReport, String>) + Send + 'static,
{
    let mut inputs = WorkerTopicInputs::new(thread_config.name.clone())?;
    let subscriber = inputs
        .subscribe::<LidarRawMessage>(&bus, topics::LIDAR_RAW)?
        .into_subscriber();
    let publisher = bus.publisher::<LidarPointCloudMessage>(topics::POINTCLOUD_DECODED)?;
    spawn_worker(thread_config, move || {
        let result = run_lidar_decoder(decoder_slot, subscriber, publisher)
            .map_err(|error| error.to_string());
        if result.is_err() {
            stop.request_stop();
        }
        on_complete(result);
    })
}

fn run_lidar_reader(
    connect_lidar: LidarConnector,
    publisher: TopicPublisher<LidarRawMessage>,
    reconnect_interval: Duration,
    decoder_slot: Arc<LidarDecoderSlot>,
    stop: StopToken,
) -> io::Result<LidarWorkerReport> {
    let mut report = LidarWorkerReport::default();
    while !stop.is_stop_requested() {
        let mut connected = false;
        let session = (|| -> io::Result<()> {
            let lidar = connect_lidar()?;
            let mut parts = lidar.into_parts();
            decoder_slot.replace(parts.decoder);
            connected = true;
            println!("LiDAR connected: {}", parts.device_name);

            // Incoming data is the health signal; no extra polling runs here.
            while !stop.is_stop_requested() {
                let packet = parts.reader.read_raw_packet()?;
                let sensor_timestamp_ns = packet.timestamp_ns();
                publisher.publish(LidarMessage::new(
                    parts.lidar_id,
                    report.message_count,
                    sensor_timestamp_ns,
                    system_timestamp_ns(),
                    packet,
                ))?;
                report.message_count += 1;
            }
            Ok(())
        })();

        if let Err(error) = session {
            if stop.is_stop_requested() {
                break;
            }
            eprintln!(
                "{}: {error}",
                if connected {
                    "LiDAR connection lost"
                } else {
                    "LiDAR connection failed"
                }
            );
        }

        if !stop.is_stop_requested() {
            eprintln!(
                "Retrying LiDAR connection in {:.3} second(s).",
                reconnect_interval.as_secs_f64()
            );
            wait_before_reconnect(reconnect_interval, &stop);
        }
    }
    Ok(report)
}

fn run_lidar_decoder(
    decoder_slot: Arc<LidarDecoderSlot>,
    subscriber: TopicSubscriber<LidarRawMessage>,
    publisher: TopicPublisher<LidarPointCloudMessage>,
) -> io::Result<LidarWorkerReport> {
    let mut report = LidarWorkerReport::default();
    loop {
        match subscriber.receive()? {
            ReceiveStatus::Message(message) => {
                let frame = decoder_slot.decode_packet(&message.payload)?;
                publisher.publish(LidarMessage::new(
                    message.lidar_id.clone(),
                    message.sequence,
                    message.sensor_timestamp_ns,
                    message.received_timestamp_ns,
                    frame,
                ))?;
                report.message_count += 1;
            }
            ReceiveStatus::Closed => break,
            ReceiveStatus::Timeout => continue,
        }
    }
    report.dropped_message_count = subscriber.dropped_messages();
    Ok(report)
}

fn wait_before_reconnect(interval: Duration, stop: &StopToken) {
    let deadline = Instant::now() + interval;
    while !stop.is_stop_requested() {
        let now = Instant::now();
        if now >= deadline {
            return;
        }
        thread::sleep(
            deadline
                .saturating_duration_since(now)
                .min(Duration::from_millis(100)),
        );
    }
}

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
