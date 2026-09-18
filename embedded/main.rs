//! Explicit startup entry point for the independent Rust workers.

mod config;

use std::{path::Path, process::ExitCode, sync::Arc, time::Instant};

use config::{validate_worker_selection, AppConfig, DEFAULT_DEVICE_CONFIG_PATH};
use vista_edge::{
    application::{
        grafana_bridge::{spawn_grafana_bridge, GrafanaBridgeReport},
        logger::{spawn_pcd_logger, spawn_raw_logger, LoggerReport},
        pointcloud_processing::{spawn_preprocessing_worker, PreprocessingReport},
        system_monitor::{spawn_system_monitor, SystemMonitorReport},
    },
    devices::lidar::{
        spawn_lidar_decode_worker, spawn_lidar_read_worker, Lidar, LidarConnector,
        LidarDecoderSlot, LidarType, LidarWorkerReport,
    },
    models::topics,
    platform::{
        self,
        message_bus::MessageBus,
        threading::{StopToken, WorkerHandle},
        workers::{
            add_worker, collect_worker_error, completion_for, failure_if_any, join_workers,
            new_worker_result, stop_and_join_noexcept, wait_for_shutdown_request,
        },
    },
};

fn run() -> Result<(), String> {
    // All user-editable device settings come from DeviceConfig.txt.
    let config = AppConfig::load()?;
    let runtime = config.runtime_config()?;
    validate_worker_selection(&config, &runtime)?;

    println!(
        "Device configuration: {} (selected {})",
        Path::new(DEFAULT_DEVICE_CONFIG_PATH).display(),
        config.lidar_type
    );
    println!("Platform: {}", platform::PLATFORM_NAME);
    match config.lidar_type {
        LidarType::RealSenseL515 => println!(
            "LiDAR configuration: realsense-l515 over USB, depth={}x{}@{} FPS, device={}",
            config.depth_width,
            config.depth_height,
            config.depth_fps,
            config
                .usb_serial
                .as_deref()
                .unwrap_or("first matching L515")
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
    println!(
        "LiDAR reconnect interval: {:.3} second(s)",
        config.lidar_reconnect_interval.as_secs_f64()
    );
    if config.grafana.enabled {
        println!(
            "Grafana Live: http://{}:{}/api/live/push/{}",
            config.grafana.host, config.grafana.port, config.grafana.namespace
        );
    } else {
        println!("Grafana Live: disabled");
    }

    let started = Instant::now();
    let stop = StopToken::new();
    let bus = Arc::new(MessageBus::new(16).map_err(|error| error.to_string())?);
    let decoder_slot = Arc::new(LidarDecoderSlot::new());
    let lidar_config = config.lidar_config();

    // Main controls queue sizes and which workers exist. Each worker creates
    // its own publishers and subscribers inside its spawn function.
    bus.configure_topic(topics::LIDAR_RAW, runtime.queues.raw_capacity)
        .map_err(|error| error.to_string())?;
    bus.configure_topic(topics::POINTCLOUD_DECODED, runtime.queues.decoded_capacity)
        .map_err(|error| error.to_string())?;
    bus.configure_topic(
        topics::POINTCLOUD_PROCESSED,
        runtime.queues.processed_capacity,
    )
    .map_err(|error| error.to_string())?;
    for topic in [
        topics::SYSTEM_HEALTH,
        topics::WORKER_HEALTH,
        topics::STORAGE_HEALTH,
        topics::POWER_THERMAL,
    ] {
        bus.configure_topic(topic, runtime.queues.telemetry_capacity)
            .map_err(|error| error.to_string())?;
    }

    let read_result = new_worker_result::<LidarWorkerReport>();
    let decode_result = new_worker_result::<LidarWorkerReport>();
    let raw_logger_result = new_worker_result::<LoggerReport>();
    let preprocessing_result = new_worker_result::<PreprocessingReport>();
    let pcd_logger_result = new_worker_result::<LoggerReport>();
    let grafana_result = new_worker_result::<GrafanaBridgeReport>();
    let system_monitor_result = new_worker_result::<SystemMonitorReport>();

    let mut workers: Vec<WorkerHandle<()>> = Vec::with_capacity(7);
    let mut read_started = false;
    let mut decode_started = false;
    let mut raw_logger_started = false;
    let mut preprocessing_started = false;
    let mut pcd_logger_started = false;
    let mut grafana_started = false;
    let mut system_monitor_started = false;

    let startup = (|| -> Result<(), String> {
        if runtime.threads.grafana_bridge.enabled && config.grafana.enabled {
            let handle = spawn_grafana_bridge(
                Arc::clone(&bus),
                runtime.threads.grafana_bridge.thread.clone(),
                stop.clone(),
                config.grafana.clone(),
                completion_for(Arc::clone(&grafana_result)),
            )
            .map_err(|error| error.to_string())?;
            add_worker(&mut workers, handle);
            grafana_started = true;
        }

        if runtime.threads.system_monitor.enabled {
            let handle = spawn_system_monitor(
                Arc::clone(&bus),
                runtime.threads.system_monitor.thread.clone(),
                stop.clone(),
                config.system_monitor.clone(),
                completion_for(Arc::clone(&system_monitor_result)),
            )
            .map_err(|error| error.to_string())?;
            add_worker(&mut workers, handle);
            system_monitor_started = true;
        }

        if runtime.threads.pcd_logger.enabled {
            if let Some(path) = config.pcd_path.clone() {
                let handle = spawn_pcd_logger(
                    Arc::clone(&bus),
                    runtime.threads.pcd_logger.thread.clone(),
                    stop.clone(),
                    path,
                    completion_for(Arc::clone(&pcd_logger_result)),
                )
                .map_err(|error| error.to_string())?;
                add_worker(&mut workers, handle);
                pcd_logger_started = true;
            }
        }

        if runtime.threads.preprocessing.enabled {
            let handle = spawn_preprocessing_worker(
                Arc::clone(&bus),
                runtime.threads.preprocessing.thread.clone(),
                stop.clone(),
                runtime.preprocessing,
                completion_for(Arc::clone(&preprocessing_result)),
            )
            .map_err(|error| error.to_string())?;
            add_worker(&mut workers, handle);
            preprocessing_started = true;
        }

        if runtime.threads.lidar_decode.enabled {
            let handle = spawn_lidar_decode_worker(
                Arc::clone(&bus),
                runtime.threads.lidar_decode.thread.clone(),
                stop.clone(),
                Arc::clone(&decoder_slot),
                completion_for(Arc::clone(&decode_result)),
            )
            .map_err(|error| error.to_string())?;
            add_worker(&mut workers, handle);
            decode_started = true;
        }

        if runtime.threads.raw_logger.enabled {
            if let Some(path) = config.raw_path.clone() {
                let handle = spawn_raw_logger(
                    Arc::clone(&bus),
                    runtime.threads.raw_logger.thread.clone(),
                    stop.clone(),
                    path,
                    completion_for(Arc::clone(&raw_logger_result)),
                )
                .map_err(|error| error.to_string())?;
                add_worker(&mut workers, handle);
                raw_logger_started = true;
            }
        }

        if runtime.threads.lidar_read.enabled {
            let connector_config = lidar_config.clone();
            let connector: LidarConnector = Arc::new(move || Lidar::connect(&connector_config));
            let handle = spawn_lidar_read_worker(
                Arc::clone(&bus),
                runtime.threads.lidar_read.thread.clone(),
                stop.clone(),
                connector,
                config.lidar_reconnect_interval,
                Arc::clone(&decoder_slot),
                completion_for(Arc::clone(&read_result)),
            )
            .map_err(|error| error.to_string())?;
            add_worker(&mut workers, handle);
            read_started = true;
        }
        Ok(())
    })();

    if let Err(error) = startup {
        stop_and_join_noexcept(&mut workers, &stop, &bus);
        return Err(error);
    }

    println!("Capture is running continuously. Press Ctrl+C to stop.");
    if let Err(error) = wait_for_shutdown_request(&stop) {
        stop_and_join_noexcept(&mut workers, &stop, &bus);
        return Err(error.to_string());
    }

    stop.request_stop();
    bus.close();
    let mut failures = Vec::new();
    join_workers(&mut workers, &stop, &bus, &mut failures);
    collect_worker_error(
        &mut failures,
        &runtime.threads.lidar_read.thread.name,
        &read_result,
        read_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.lidar_decode.thread.name,
        &decode_result,
        decode_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.raw_logger.thread.name,
        &raw_logger_result,
        raw_logger_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.preprocessing.thread.name,
        &preprocessing_result,
        preprocessing_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.pcd_logger.thread.name,
        &pcd_logger_result,
        pcd_logger_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.grafana_bridge.thread.name,
        &grafana_result,
        grafana_started,
    );
    collect_worker_error(
        &mut failures,
        &runtime.threads.system_monitor.thread.name,
        &system_monitor_result,
        system_monitor_started,
    );
    failure_if_any(failures).map_err(|error| error.to_string())?;

    let read = read_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let decode = decode_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let raw_logger = raw_logger_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let preprocessing = preprocessing_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let pcd_logger = pcd_logger_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let grafana = grafana_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let system_monitor = system_monitor_result
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());

    println!(
        "Capture complete: {} packet(s), {} processed point(s), {:.2} seconds",
        read.report
            .as_ref()
            .map_or(0, |report| report.message_count),
        preprocessing
            .report
            .as_ref()
            .map_or(0, |report| report.point_count),
        started.elapsed().as_secs_f64()
    );
    println!(
        "Ring-buffer drops: decoder={}, raw-logger={}, preprocessing={}, pcd-logger={}",
        decode
            .report
            .as_ref()
            .map_or(0, |report| report.dropped_message_count),
        raw_logger
            .report
            .as_ref()
            .map_or(0, |report| report.dropped_message_count),
        preprocessing
            .report
            .as_ref()
            .map_or(0, |report| report.dropped_message_count),
        pcd_logger
            .report
            .as_ref()
            .map_or(0, |report| report.dropped_message_count)
    );
    if let Some(report) = &grafana.report {
        println!(
            "Grafana Live: published={}, failed-attempts={}, input-drops={}",
            report.published_measurements,
            report.failed_publish_attempts,
            report.dropped_input_messages
        );
    }
    if let Some(report) = &system_monitor.report {
        println!("System monitor: {} sample(s)", report.sample_count);
    }
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
