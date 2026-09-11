//! Command-line entry point for a configurable LiDAR capture session.

use std::{
    env,
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
    devices::lidar::{Lidar, LidarConfig, LidarType},
    platform,
};

// Default capture values are kept together in main.rs.
const DEFAULT_LIDAR_TYPE: LidarType = LidarType::QuanergyM8;
const DEFAULT_SENSOR_IP: IpAddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 3));
// None asks the selected driver to use its own default port.
const DEFAULT_TCP_PORT: Option<u16> = None;
const DEFAULT_DURATION_SECONDS: u64 = 10;
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
    raw_path: Option<PathBuf>,
    pcd_path: Option<PathBuf>,
}

impl Default for AppConfig {
    /// Creates a default capture that reads and processes data without saving files.
    fn default() -> Self {
        Self {
            lidar_type: DEFAULT_LIDAR_TYPE,
            sensor_ip: DEFAULT_SENSOR_IP,
            tcp_port: DEFAULT_TCP_PORT,
            duration_seconds: DEFAULT_DURATION_SECONDS,
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
         cargo run -- [lidar-type] [sensor-ip] [tcp-port] [duration-seconds] [*.bin] [*.pcd]\n\
         cargo run -- [--lidar TYPE] [--ip IP] [--port PORT] [--duration SECONDS] \
         [--raw FILE.bin] [--pcd FILE.pcd]\n\
         \n\
         Defaults: lidar={DEFAULT_LIDAR_TYPE}, ip={DEFAULT_SENSOR_IP}, \
         port=<driver default>, duration={DEFAULT_DURATION_SECONDS}, raw=<disabled>, pcd=<disabled>"
    );
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
fn parse_positional(args: &[String]) -> Result<AppConfig, String> {
    if args.len() > 6 {
        return Err("too many positional arguments; use --help for syntax".to_owned());
    }

    let mut config = AppConfig::default();
    if let Some(value) = args.first() {
        config.lidar_type = value.parse::<LidarType>()?;
    }
    if let Some(value) = args.get(1) {
        config.sensor_ip = value
            .parse::<IpAddr>()
            .map_err(|error| format!("invalid sensor IP '{value}': {error}"))?;
    }
    if let Some(value) = args.get(2) {
        config.tcp_port = Some(parse_port(value)?);
    }
    if let Some(value) = args.get(3) {
        config.duration_seconds = parse_duration(value)?;
    }
    for value in args.iter().skip(4) {
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
fn parse_named(args: &[String]) -> Result<AppConfig, String> {
    let mut config = AppConfig::default();
    let mut index = 0;

    while index < args.len() {
        let option = args[index].as_str();
        let value = option_value(args, index, option)?;

        match option {
            "--lidar" => config.lidar_type = value.parse::<LidarType>()?,
            "--ip" => {
                config.sensor_ip = value
                    .parse::<IpAddr>()
                    .map_err(|error| format!("invalid sensor IP '{value}': {error}"))?;
            }
            "--port" => config.tcp_port = Some(parse_port(value)?),
            "--duration" => config.duration_seconds = parse_duration(value)?,
            "--raw" => config.raw_path = Some(output_path(value, "bin")?),
            "--pcd" => config.pcd_path = Some(output_path(value, "pcd")?),
            _ => return Err(format!("unknown option '{option}'; use --help for syntax")),
        }

        index += 2;
    }
    Ok(config)
}

/// Selects positional or named parsing without allowing ambiguous mixed syntax.
fn parse_arguments(args: &[String]) -> Result<AppConfig, String> {
    if args.first().is_some_and(|value| value.starts_with("--")) {
        return parse_named(args);
    }
    if args.iter().any(|value| value.starts_with("--")) {
        return Err("do not mix positional values with named options".to_owned());
    }
    parse_positional(args)
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

    let config = parse_arguments(&args)?;
    let mut lidar_config = LidarConfig::new(config.lidar_type, config.sensor_ip);
    if let Some(port) = config.tcp_port {
        lidar_config = lidar_config.with_port(port);
    }

    println!("Platform: {}", platform::PLATFORM_NAME);
    match config.tcp_port {
        Some(port) => println!(
            "LiDAR configuration: {} at {}:{}",
            config.lidar_type, config.sensor_ip, port
        ),
        None => println!(
            "LiDAR configuration: {} at {}, using driver default port",
            config.lidar_type, config.sensor_ip
        ),
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
