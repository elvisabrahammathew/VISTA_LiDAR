//! Minimal startup entry point for a configurable LiDAR capture session.

mod config;

use std::{env, path::Path, process::ExitCode};

use config::{help_requested, print_usage, AppConfig, DEFAULT_DEVICE_CONFIG_PATH};
use vista_lidar::{
    application::supervisor,
    devices::lidar::{Lidar, LidarType},
    platform,
};

/// Creates the selected driver and supervises one capture session.
fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if help_requested(&args) {
        print_usage();
        return Ok(());
    }

    // All defaults and overrides are resolved by config.rs before hardware starts.
    let config = AppConfig::load(&args)?;
    println!(
        "Device configuration: {} (selected {})",
        Path::new(DEFAULT_DEVICE_CONFIG_PATH).display(),
        config.lidar_type
    );
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

    // Driver selection happens once; workers only use the common LiDAR interfaces.
    let lidar = Lidar::connect(&config.lidar_config()).map_err(|error| error.to_string())?;
    println!("Connected driver: {}", lidar.device_name());

    // The supervisor moves Reader and Decoder into their independent threads.
    let stats = supervisor::run_capture(lidar, config.capture_config()?)
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
