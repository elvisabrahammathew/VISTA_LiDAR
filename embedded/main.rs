use std::{
    env,
    net::{IpAddr, SocketAddr},
    path::PathBuf,
    process::ExitCode,
    time::Duration,
};

use vista_lidar::{application::pipeline, platform};

const DEFAULT_SENSOR_IP: &str = "192.168.1.3";
const DEFAULT_PORT: u16 = 4141;
const DEFAULT_DURATION_SECONDS: u64 = 10;
const DEFAULT_RAW_PATH: &str = "data/raw/quanergy_m8.bin";
const DEFAULT_PCD_PATH: &str = "data/processed/quanergy_m8.pcd";

fn usage() {
    println!(
        "Usage: quanergy-m8-capture [sensor-ip] [duration-seconds] [raw-file] [pcd-file]\n\
         Defaults: {DEFAULT_SENSOR_IP} {DEFAULT_DURATION_SECONDS} \
         {DEFAULT_RAW_PATH} {DEFAULT_PCD_PATH}"
    );
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let sensor_ip = args.next().unwrap_or_else(|| DEFAULT_SENSOR_IP.to_owned());

    if sensor_ip == "--help" || sensor_ip == "-h" {
        usage();
        return Ok(());
    }

    let duration_seconds = args
        .next()
        .map(|value| {
            value
                .parse::<u64>()
                .map_err(|error| format!("invalid duration '{value}': {error}"))
        })
        .transpose()?
        .unwrap_or(DEFAULT_DURATION_SECONDS);

    if duration_seconds == 0 {
        return Err("duration must be greater than zero".to_owned());
    }

    let raw_path = PathBuf::from(args.next().unwrap_or_else(|| DEFAULT_RAW_PATH.to_owned()));
    let pcd_path = PathBuf::from(args.next().unwrap_or_else(|| DEFAULT_PCD_PATH.to_owned()));

    if args.next().is_some() {
        return Err("too many arguments; use --help for syntax".to_owned());
    }

    let ip = sensor_ip
        .parse::<IpAddr>()
        .map_err(|error| format!("invalid sensor IP '{sensor_ip}': {error}"))?;
    let address = SocketAddr::new(ip, DEFAULT_PORT);

    println!("Platform: {}", platform::PLATFORM_NAME);
    println!("Connecting to {address}");
    println!("Raw output: {}", raw_path.display());
    println!("Point-cloud output: {}", pcd_path.display());

    let stats = pipeline::capture_quanergy_m8(
        address,
        Duration::from_secs(duration_seconds),
        &raw_path,
        &pcd_path,
    )
    .map_err(|error| error.to_string())?;

    println!(
        "Capture complete: {} packet(s), {} point(s), {:.2} seconds",
        stats.packet_count,
        stats.point_count,
        stats.elapsed.as_secs_f64()
    );

    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("Error: {message}");
            ExitCode::FAILURE
        }
    }
}
