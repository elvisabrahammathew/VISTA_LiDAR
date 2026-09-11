//! Command-line entry point for a configurable LiDAR capture session.

use std::{
    env, fs,
    net::{IpAddr, Ipv4Addr},
    path::{Path, PathBuf},
    process::ExitCode,
    time::Duration,
};

use vista_lidar::{
    application::{
        pipeline,
        pointcloud_processing::{GroundRemovalConfig, PreprocessingConfig},
    },
    devices::lidar::{DepthStreamConfig, Lidar, LidarConfig, LidarType},
    platform,
};

// Default capture values are kept together in main.rs.
const DEFAULT_DEVICE_CONFIG_PATH: &str = "DeviceConfig.txt";
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

const MIN_DISTANCE_METERS: f32 = 0.1;
const MAX_DISTANCE_METERS: f32 = 200.0;
const VOXEL_SIZE_METERS: Option<f32> = Some(0.05);
// Set this to Some(-sensor_mounting_height) when the ground Z value is known.
const GROUND_HEIGHT_METERS: Option<f32> = None;
const GROUND_TOLERANCE_METERS: f32 = 0.15;

/// Holds the final settings after defaults and command-line overrides are applied.
#[derive(Debug, Clone)]
struct AppConfig {
    lidar_type: LidarType,
    sensor_ip: IpAddr,
    tcp_port: Option<u16>,
    duration_seconds: u64,
    depth_width: u32,
    depth_height: u32,
    depth_fps: u32,
    raw_path: Option<PathBuf>,
    pcd_path: Option<PathBuf>,
}

impl AppConfig {
    /// Creates a capture configuration for the LiDAR selected in DeviceConfig.txt.
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
}

/// Prints positional and named command forms supported by the application.
fn usage() {
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

/// Parses the LiDAR selection while accepting an unused Radar entry for future work.
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
            // Radar selection is reserved for a future driver and is intentionally ignored.
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

/// Reads the device selection from the text configuration file.
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

/// Parses and validates a positive RealSense stream dimension or frame rate.
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

/// Assigns a positional output according to its .bin or .pcd extension.
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

/// Parses the complete positional form while allowing output files to be omitted.
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

/// Returns the value that follows a named option.
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

/// Builds the sensor-neutral preprocessing configuration used by the pipeline.
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

/// Creates the selected driver and runs one capture session.
fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.iter().any(|value| value == "--help" || value == "-h") {
        usage();
        return Ok(());
    }

    let device_config_path = Path::new(DEFAULT_DEVICE_CONFIG_PATH);
    let lidar_type = load_device_config(device_config_path)?;
    let config = parse_arguments(&args, lidar_type)?;
    println!(
        "Device configuration: {} (selected {})",
        device_config_path.display(),
        config.lidar_type
    );
    let mut lidar_config = LidarConfig::new(config.lidar_type, config.sensor_ip);
    if let Some(port) = config.tcp_port {
        lidar_config = lidar_config.with_port(port);
    }
    if config.lidar_type == LidarType::RealSenseL515 {
        lidar_config = lidar_config.with_depth_stream(DepthStreamConfig::new(
            config.depth_width,
            config.depth_height,
            config.depth_fps,
            Duration::from_millis(DEFAULT_FRAME_TIMEOUT_MILLISECONDS),
        ));
    }

    println!("Platform: {}", platform::PLATFORM_NAME);
    match config.lidar_type {
        LidarType::RealSenseL515 => println!(
            "LiDAR configuration: {} over USB, depth={}x{}@{} FPS",
            config.lidar_type, config.depth_width, config.depth_height, config.depth_fps
        ),
        _ => match config.tcp_port {
            Some(port) => println!(
                "LiDAR configuration: {} at {}:{}",
                config.lidar_type, config.sensor_ip, port
            ),
            None => println!(
                "LiDAR configuration: {} at {}, using driver default port",
                config.lidar_type, config.sensor_ip
            ),
        },
    }
    println!(
        "Raw output: {}",
        config
            .raw_path
            .as_ref()
            .map_or_else(|| "disabled".to_owned(), |path| path.display().to_string())
    );
    println!(
        "Point-cloud output: {}",
        config
            .pcd_path
            .as_ref()
            .map_or_else(|| "disabled".to_owned(), |path| path.display().to_string())
    );

    // Driver selection happens once; the pipeline only uses the common Lidar interface.
    let mut lidar = Lidar::connect(&lidar_config).map_err(|error| error.to_string())?;
    println!("Connected driver: {}", lidar.device_name());

    let stats = pipeline::capture_lidar(
        &mut lidar,
        Duration::from_secs(config.duration_seconds),
        config.raw_path.as_deref(),
        config.pcd_path.as_deref(),
        &preprocessing_config(),
    )
    .map_err(|error| error.to_string())?;

    println!(
        "Capture complete: {} packet(s), {} processed point(s), {:.2} seconds",
        stats.packet_count,
        stats.point_count,
        stats.elapsed.as_secs_f64()
    );
    Ok(())
}

/// Converts the application result into a process exit code.
fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("Error: {message}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
#[path = "unittest/main_test.rs"]
mod tests;
