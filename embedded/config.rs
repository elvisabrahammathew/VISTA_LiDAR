//! Loads device selection, command-line overrides, and capture runtime settings.

use std::{
    fs,
    net::{IpAddr, Ipv4Addr},
    path::{Path, PathBuf},
    time::Duration,
};

use vista_lidar::{
    application::{
        pointcloud_processing::{GroundRemovalConfig, PreprocessingConfig},
        supervisor::{CaptureConfig, CaptureThreadConfig, TopicQueueConfig},
    },
    devices::lidar::{DepthStreamConfig, LidarConfig, LidarType},
    platform::threading::ThreadConfig,
};

pub(crate) const DEFAULT_DEVICE_CONFIG_PATH: &str = "DeviceConfig.txt";
const DEFAULT_SENSOR_IP: IpAddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 3));
// None asks the selected driver to use its own default port.
const DEFAULT_TCP_PORT: Option<u16> = None;
const DEFAULT_DURATION_SECONDS: u64 = 10;
const DEFAULT_DEPTH_WIDTH: u32 = 640;
const DEFAULT_DEPTH_HEIGHT: u32 = 480;
const DEFAULT_DEPTH_FPS: u32 = 30;
const DEFAULT_FRAME_TIMEOUT_MILLISECONDS: u64 = 5_000;
// None disables the corresponding local output file.
const DEFAULT_RAW_PATH: Option<&str> = None;
const DEFAULT_PCD_PATH: Option<&str> = None;

// Project priorities use 1 as highest and 5 as lowest. These values are kept
// together so deployment tuning never changes the worker implementation.
const LIDAR_READ_PRIORITY: u8 = 1;
const LIDAR_DECODE_PRIORITY: u8 = 2;
const RAW_LOGGER_PRIORITY: u8 = 2;
const PCD_LOGGER_PRIORITY: u8 = 2;
const PREPROCESSING_PRIORITY: u8 = 3;

// Bounded queues protect Windows and Linux hosts from unlimited memory growth
// if decoding, preprocessing, or disk output temporarily falls behind.
const RAW_TOPIC_CAPACITY: usize = 32;
const DECODED_POINTCLOUD_TOPIC_CAPACITY: usize = 8;
const PROCESSED_POINTCLOUD_TOPIC_CAPACITY: usize = 8;

const MIN_DISTANCE_METERS: f32 = 0.1;
const MAX_DISTANCE_METERS: f32 = 200.0;
const VOXEL_SIZE_METERS: Option<f32> = Some(0.05);
// Change to Some(-2.5) only after confirming the installed LiDAR Z axis.
const GROUND_HEIGHT_METERS: Option<f32> = None;
const GROUND_TOLERANCE_METERS: f32 = 0.15;

/// Holds final settings after file defaults and command-line overrides are applied.
#[derive(Debug, Clone)]
pub(crate) struct AppConfig {
    pub lidar_type: LidarType,
    pub sensor_ip: IpAddr,
    pub tcp_port: Option<u16>,
    pub duration_seconds: u64,
    pub depth_width: u32,
    pub depth_height: u32,
    pub depth_fps: u32,
    pub raw_path: Option<PathBuf>,
    pub pcd_path: Option<PathBuf>,
}

impl AppConfig {
    /// Loads the LiDAR type from DeviceConfig.txt and applies command-line values.
    pub(crate) fn load(args: &[String]) -> Result<Self, String> {
        let lidar_type = load_device_config(Path::new(DEFAULT_DEVICE_CONFIG_PATH))?;
        parse_arguments(args, lidar_type)
    }

    /// Creates a capture configuration for the selected LiDAR using defaults.
    fn new(lidar_type: LidarType) -> Self {
        Self {
            lidar_type,
            sensor_ip: DEFAULT_SENSOR_IP,
            tcp_port: DEFAULT_TCP_PORT,
            duration_seconds: DEFAULT_DURATION_SECONDS,
            depth_width: DEFAULT_DEPTH_WIDTH,
            depth_height: DEFAULT_DEPTH_HEIGHT,
            depth_fps: DEFAULT_DEPTH_FPS,
            raw_path: DEFAULT_RAW_PATH.map(PathBuf::from),
            pcd_path: DEFAULT_PCD_PATH.map(PathBuf::from),
        }
    }

    /// Converts application settings into the selected driver's startup values.
    pub(crate) fn lidar_config(&self) -> LidarConfig {
        let mut config = LidarConfig::new(self.lidar_type, self.sensor_ip);
        if let Some(port) = self.tcp_port {
            config = config.with_port(port);
        }
        if self.lidar_type == LidarType::RealSenseL515 {
            config = config.with_depth_stream(DepthStreamConfig::new(
                self.depth_width,
                self.depth_height,
                self.depth_fps,
                Duration::from_millis(DEFAULT_FRAME_TIMEOUT_MILLISECONDS),
            ));
        }
        config
    }

    /// Builds worker, queue, output, and preprocessing settings for the supervisor.
    pub(crate) fn capture_config(&self) -> Result<CaptureConfig, String> {
        Ok(CaptureConfig {
            duration: Duration::from_secs(self.duration_seconds),
            raw_path: self.raw_path.clone(),
            pointcloud_path: self.pcd_path.clone(),
            preprocessing: preprocessing_config(),
            threads: CaptureThreadConfig {
                lidar_read: thread_config("lidar-read", LIDAR_READ_PRIORITY)?,
                lidar_decode: thread_config("lidar-decode", LIDAR_DECODE_PRIORITY)?,
                raw_logger: thread_config("raw-logger", RAW_LOGGER_PRIORITY)?,
                preprocessing: thread_config("pointcloud-preprocessing", PREPROCESSING_PRIORITY)?,
                pcd_logger: thread_config("pcd-logger", PCD_LOGGER_PRIORITY)?,
            },
            queues: TopicQueueConfig {
                raw_capacity: RAW_TOPIC_CAPACITY,
                decoded_capacity: DECODED_POINTCLOUD_TOPIC_CAPACITY,
                processed_capacity: PROCESSED_POINTCLOUD_TOPIC_CAPACITY,
            },
        })
    }
}

/// Reports whether the command line only requests usage information.
pub(crate) fn help_requested(args: &[String]) -> bool {
    args.iter().any(|value| value == "--help" || value == "-h")
}

/// Prints positional and named command forms supported by the application.
pub(crate) fn print_usage() {
    println!(
        "Usage:\n\
         \n\
         cargo run\n\
         cargo run -- [sensor-ip] [tcp-port] [duration-seconds] [*.bin] [*.pcd]\n\
         cargo run -- [--ip IP] [--port PORT] [--duration SECONDS] \
         [--width PIXELS] [--height PIXELS] [--fps RATE] \
         [--raw FILE.bin] [--pcd FILE.pcd]\n\
         \n\
         Device selection: {DEFAULT_DEVICE_CONFIG_PATH} (Lidar: quanergy-m8 | realsense-l515)\n\
         Defaults: ip={DEFAULT_SENSOR_IP}, \
         port=<driver default>, duration={DEFAULT_DURATION_SECONDS}, \
         L515={DEFAULT_DEPTH_WIDTH}x{DEFAULT_DEPTH_HEIGHT}@{DEFAULT_DEPTH_FPS}, \
         raw=<disabled>, pcd=<disabled>"
    );
}

/// Creates a validated named worker configuration.
fn thread_config(name: &str, priority: u8) -> Result<ThreadConfig, String> {
    ThreadConfig::new(name, priority).map_err(|error| error.to_string())
}

/// Parses LiDAR selection while reserving the Radar entry for future work.
fn parse_device_config(contents: &str) -> Result<LidarType, String> {
    let mut lidar_type = None;

    for (index, original_line) in contents.lines().enumerate() {
        let line = original_line
            .trim_start_matches('\u{feff}')
            .split('#')
            .next()
            .unwrap_or_default()
            .trim();
        if line.is_empty() {
            continue;
        }

        let (key, value) = line.split_once(':').ok_or_else(|| {
            format!(
                "invalid device configuration at line {}: expected 'Key: value'",
                index + 1
            )
        })?;
        let key = key.trim();
        let value = value.trim();

        if key.eq_ignore_ascii_case("lidar") {
            if lidar_type.is_some() {
                return Err("DeviceConfig.txt contains more than one Lidar entry".to_owned());
            }
            if value.is_empty() || value.eq_ignore_ascii_case("none") {
                return Err(
                    "DeviceConfig.txt must select a LiDAR: quanergy-m8 or realsense-l515"
                        .to_owned(),
                );
            }
            lidar_type = Some(value.parse::<LidarType>()?);
        } else if key.eq_ignore_ascii_case("radar") {
            // Radar selection is accepted but remains inactive until its driver exists.
        } else {
            return Err(format!(
                "unknown device configuration key '{key}' at line {}",
                index + 1
            ));
        }
    }

    lidar_type.ok_or_else(|| {
        "DeviceConfig.txt is missing 'Lidar: quanergy-m8' or 'Lidar: realsense-l515'".to_owned()
    })
}

/// Reads the device-selection text file.
fn load_device_config(path: &Path) -> Result<LidarType, String> {
    let contents = fs::read_to_string(path).map_err(|error| {
        format!(
            "could not read device configuration '{}': {error}",
            path.display()
        )
    })?;
    parse_device_config(&contents)
}

/// Parses and validates a TCP port.
fn parse_port(value: &str) -> Result<u16, String> {
    let port = value
        .parse::<u16>()
        .map_err(|error| format!("invalid TCP port '{value}': {error}"))?;
    if port == 0 {
        return Err("TCP port must be greater than zero".to_owned());
    }
    Ok(port)
}

/// Parses and validates a positive capture duration.
fn parse_duration(value: &str) -> Result<u64, String> {
    let duration = value
        .parse::<u64>()
        .map_err(|error| format!("invalid duration '{value}': {error}"))?;
    if duration == 0 {
        return Err("duration must be greater than zero".to_owned());
    }
    Ok(duration)
}

/// Parses and validates a positive RealSense dimension or frame rate.
fn parse_positive_u32(value: &str, label: &str) -> Result<u32, String> {
    let parsed = value
        .parse::<u32>()
        .map_err(|error| format!("invalid {label} '{value}': {error}"))?;
    if parsed == 0 {
        return Err(format!("{label} must be greater than zero"));
    }
    Ok(parsed)
}

/// Verifies a file extension before enabling an output path.
fn output_path(value: &str, expected_extension: &str) -> Result<PathBuf, String> {
    let path = PathBuf::from(value);
    let extension = path
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default();

    if !extension.eq_ignore_ascii_case(expected_extension) {
        return Err(format!(
            "output file '{}' must have the .{expected_extension} extension",
            path.display()
        ));
    }
    Ok(path)
}

/// Assigns a positional output according to its `.bin` or `.pcd` extension.
fn assign_output(config: &mut AppConfig, value: &str) -> Result<(), String> {
    let extension = Path::new(value)
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default();

    if extension.eq_ignore_ascii_case("bin") {
        if config.raw_path.is_some() {
            return Err("more than one RAW .bin file was provided".to_owned());
        }
        config.raw_path = Some(PathBuf::from(value));
    } else if extension.eq_ignore_ascii_case("pcd") {
        if config.pcd_path.is_some() {
            return Err("more than one point-cloud .pcd file was provided".to_owned());
        }
        config.pcd_path = Some(PathBuf::from(value));
    } else {
        return Err(format!(
            "output file '{value}' must have a .bin or .pcd extension"
        ));
    }
    Ok(())
}

/// Parses positional values while allowing output files to be omitted.
fn parse_positional(args: &[String], lidar_type: LidarType) -> Result<AppConfig, String> {
    if args.len() > 5 {
        return Err("too many positional arguments; use --help for syntax".to_owned());
    }

    let mut config = AppConfig::new(lidar_type);
    if let Some(value) = args.first() {
        config.sensor_ip = value
            .parse::<IpAddr>()
            .map_err(|error| format!("invalid sensor IP '{value}': {error}"))?;
    }
    if let Some(value) = args.get(1) {
        config.tcp_port = Some(parse_port(value)?);
    }
    if let Some(value) = args.get(2) {
        config.duration_seconds = parse_duration(value)?;
    }
    for value in args.iter().skip(3) {
        assign_output(&mut config, value)?;
    }
    Ok(config)
}

/// Returns the value following a named command-line option.
fn option_value<'a>(args: &'a [String], index: usize, option: &str) -> Result<&'a str, String> {
    args.get(index + 1)
        .map(String::as_str)
        .ok_or_else(|| format!("missing value after {option}"))
}

/// Parses named options so individual network or output values may be omitted.
fn parse_named(args: &[String], lidar_type: LidarType) -> Result<AppConfig, String> {
    let mut config = AppConfig::new(lidar_type);
    let mut index = 0;

    while index < args.len() {
        let option = args[index].as_str();
        let value = option_value(args, index, option)?;

        match option {
            "--ip" => {
                config.sensor_ip = value
                    .parse::<IpAddr>()
                    .map_err(|error| format!("invalid sensor IP '{value}': {error}"))?;
            }
            "--port" => config.tcp_port = Some(parse_port(value)?),
            "--duration" => config.duration_seconds = parse_duration(value)?,
            "--width" => config.depth_width = parse_positive_u32(value, "depth width")?,
            "--height" => config.depth_height = parse_positive_u32(value, "depth height")?,
            "--fps" => config.depth_fps = parse_positive_u32(value, "depth frame rate")?,
            "--raw" => config.raw_path = Some(output_path(value, "bin")?),
            "--pcd" => config.pcd_path = Some(output_path(value, "pcd")?),
            _ => return Err(format!("unknown option '{option}'; use --help for syntax")),
        }

        index += 2;
    }
    Ok(config)
}

/// Selects positional or named parsing without allowing ambiguous mixed syntax.
fn parse_arguments(args: &[String], lidar_type: LidarType) -> Result<AppConfig, String> {
    if args.first().is_some_and(|value| value.starts_with("--")) {
        return parse_named(args, lidar_type);
    }
    if args.iter().any(|value| value.starts_with("--")) {
        return Err("do not mix positional values with named options".to_owned());
    }
    parse_positional(args, lidar_type)
}

/// Builds the sensor-neutral preprocessing stages used during capture.
fn preprocessing_config() -> PreprocessingConfig {
    PreprocessingConfig {
        min_distance_m: MIN_DISTANCE_METERS,
        max_distance_m: MAX_DISTANCE_METERS,
        region_of_interest: None,
        // To enable ROI, replace None above with Some(AxisAlignedRoi { ... }).
        voxel_size_m: VOXEL_SIZE_METERS,
        ground_removal: GROUND_HEIGHT_METERS.map(|ground_height_m| GroundRemovalConfig {
            ground_height_m,
            tolerance_m: GROUND_TOLERANCE_METERS,
        }),
    }
}

#[cfg(test)]
#[path = "unittest/config_test.rs"]
mod tests;
