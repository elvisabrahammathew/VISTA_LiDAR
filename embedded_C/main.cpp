#include "lib.hpp"

int main() {
    try {
        // All user settings are loaded from DeviceConfig.txt.
        const auto config = vista::AppConfig::load();
        const auto runtime = config.runtime_config();
        vista::platform::validate_worker_selection(config, runtime);

        std::cout << "Device configuration: " << vista::default_device_config_path
                  << " (selected " << vista::devices::to_string(config.lidar_type)
                  << ")\n";
        std::cout << "Platform: " << vista::platform::platform_name() << '\n';

        if (config.lidar_type == vista::devices::LidarType::realsense_l515) {
            std::cout << "LiDAR configuration: realsense-l515 over USB, depth="
                      << config.depth_width << 'x' << config.depth_height << '@'
                      << config.depth_fps << " FPS, device="
                      << (config.usb_serial ? *config.usb_serial
                                            : "first matching L515")
                      << '\n';
        } else if (config.lidar_type == vista::devices::LidarType::unitree_l2) {
            std::cout << "LiDAR configuration: unitree-l2, mode="
                      << vista::devices::to_string(
                             config.unitree_connection_mode)
                      << ", serial=" << config.unitree_serial_port << '@'
                      << config.unitree_baud_rate << ", UDP sensor="
                      << config.sensor_ip << ':'
                      << config.sensor_port.value_or(6101) << ", local="
                      << config.unitree_local_ip << ':'
                      << config.unitree_local_port.value_or(6201) << '\n';
        } else {
            std::cout << "LiDAR configuration: "
                      << vista::devices::to_string(config.lidar_type) << " at "
                      << config.sensor_ip;
            if (config.sensor_port) {
                std::cout << ':' << *config.sensor_port;
            } else {
                std::cout << ", using driver default port";
            }
            std::cout << '\n';
        }

        std::cout << "Raw output: "
                  << (config.raw_path ? config.raw_path->string() : "disabled")
                  << '\n';
        std::cout << "Point-cloud output: "
                  << (config.pcd_path ? config.pcd_path->string() : "disabled")
                  << '\n';
        std::cout << "LiDAR reconnect interval: "
                  << static_cast<double>(config.lidar_reconnect_interval.count()) /
                         1000.0
                  << " second(s)\n";
        if (config.ground_removal) {
            const auto& ground = *config.ground_removal;
            std::cout << "Ground removal: mode="
                      << vista::application::to_string(ground.mode)
                      << ", mount=(" << ground.mount_x_m << ", "
                      << ground.mount_y_m << ", " << ground.mount_z_m
                      << ") m, RPY=(" << ground.mount_roll_deg << ", "
                      << ground.mount_pitch_deg << ", "
                      << ground.mount_yaw_deg << ") deg, floor Z="
                      << ground.floor_z_m << " m, IMU="
                      << (ground.use_imu ? "preferred with pose fallback"
                                         : "disabled")
                      << '\n';
        } else {
            std::cout << "Ground removal: disabled\n";
        }
        std::cout << "Grafana Live: ";
        if (config.grafana.enabled) {
            std::cout << "http://" << config.grafana.host << ':'
                      << config.grafana.port << "/api/live/push/"
                      << config.grafana.name_space << '\n';
        } else {
            std::cout << "disabled\n";
        }
        std::cout << "Point-cloud WebSocket: ";
        if (config.pointcloud_websocket.enabled) {
            std::cout << "ws://" << config.pointcloud_websocket.bind_address
                      << ':' << config.pointcloud_websocket.port
                      << " (maximum "
                      << config.pointcloud_websocket.maximum_points
                      << " points/frame, every "
                      << config.pointcloud_websocket.publish_interval.count()
                      << " ms)\n";
        } else {
            std::cout << "disabled\n";
        }

        const auto started = std::chrono::steady_clock::now();
        vista::platform::StopToken stop;
        vista::platform::MessageBus bus;
        auto decoder_slot =
            std::make_shared<vista::devices::LidarDecoderSlot>();
        const auto lidar_config = config.lidar_config();

        // Main only defines queue capacities. Each worker creates its own
        // publisher/subscriber endpoints inside its spawn_* function.
        bus.configure_topic(
            vista::models::topics::lidar_raw,
            runtime.queues.raw_capacity);
        bus.configure_topic(
            vista::models::topics::lidar_imu,
            runtime.queues.decoded_capacity);
        bus.configure_topic(
            vista::models::topics::pointcloud_decoded,
            runtime.queues.decoded_capacity);
        bus.configure_topic(
            vista::models::topics::pointcloud_processed,
            runtime.queues.processed_capacity);
        bus.configure_topic(
            vista::models::topics::ground_status,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::system_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::worker_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::storage_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::power_thermal,
            runtime.queues.telemetry_capacity);

        auto read_result = std::make_shared<
            vista::platform::WorkerResult<vista::devices::LidarWorkerReport>>();
        auto decode_result = std::make_shared<
            vista::platform::WorkerResult<vista::devices::LidarWorkerReport>>();
        auto raw_logger_result = std::make_shared<
            vista::platform::WorkerResult<vista::application::LoggerReport>>();
        auto preprocessing_result = std::make_shared<
            vista::platform::WorkerResult<
                vista::application::PreprocessingReport>>();
        auto pcd_logger_result = std::make_shared<
            vista::platform::WorkerResult<vista::application::LoggerReport>>();
        auto grafana_result = std::make_shared<
            vista::platform::WorkerResult<
                vista::application::GrafanaBridgeReport>>();
        auto system_monitor_result = std::make_shared<
            vista::platform::WorkerResult<
                vista::application::SystemMonitorReport>>();
        auto pointcloud_websocket_result = std::make_shared<
            vista::platform::WorkerResult<
                vista::application::PointCloudWebSocketReport>>();

        std::vector<vista::platform::WorkerHandle> workers;
        workers.reserve(8);
        bool read_started = false;
        bool decode_started = false;
        bool raw_logger_started = false;
        bool preprocessing_started = false;
        bool pcd_logger_started = false;
        bool grafana_started = false;
        bool system_monitor_started = false;
        bool pointcloud_websocket_started = false;

        try {
            // The user controls worker creation and startup order in main.
            if (runtime.threads.grafana_bridge.enabled &&
                config.grafana.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_grafana_bridge(
                        bus,
                        runtime.threads.grafana_bridge.thread,
                        stop,
                        config.grafana,
                        vista::platform::completion_for(grafana_result)));
                grafana_started = true;
            }

            if (runtime.threads.pointcloud_websocket.enabled &&
                config.pointcloud_websocket.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_pointcloud_websocket(
                        bus,
                        runtime.threads.pointcloud_websocket.thread,
                        stop,
                        config.pointcloud_websocket,
                        vista::platform::completion_for(
                            pointcloud_websocket_result)));
                pointcloud_websocket_started = true;
            }

            if (runtime.threads.system_monitor.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_system_monitor(
                        bus,
                        runtime.threads.system_monitor.thread,
                        stop,
                        config.system_monitor,
                        vista::platform::completion_for(system_monitor_result)));
                system_monitor_started = true;
            }

            if (runtime.threads.pcd_logger.enabled && config.pcd_path) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_pcd_logger(
                        bus,
                        runtime.threads.pcd_logger.thread,
                        stop,
                        *config.pcd_path,
                        vista::platform::completion_for(pcd_logger_result)));
                pcd_logger_started = true;
            }

            if (runtime.threads.preprocessing.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_preprocessing_worker(
                        bus,
                        runtime.threads.preprocessing.thread,
                        stop,
                        runtime.preprocessing,
                        vista::platform::completion_for(preprocessing_result)));
                preprocessing_started = true;
            }

            if (runtime.threads.lidar_decode.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::devices::spawn_lidar_decode_worker(
                        bus,
                        runtime.threads.lidar_decode.thread,
                        stop,
                        decoder_slot,
                        vista::platform::completion_for(decode_result)));
                decode_started = true;
            }

            if (runtime.threads.raw_logger.enabled && config.raw_path) {
                vista::platform::add_worker(
                    workers,
                    vista::application::spawn_raw_logger(
                        bus,
                        runtime.threads.raw_logger.thread,
                        stop,
                        *config.raw_path,
                        vista::platform::completion_for(raw_logger_result)));
                raw_logger_started = true;
            }

            if (runtime.threads.lidar_read.enabled) {
                vista::platform::add_worker(
                    workers,
                    vista::devices::spawn_lidar_read_worker(
                        bus,
                        runtime.threads.lidar_read.thread,
                        stop,
                        [lidar_config]() {
                            return vista::devices::Lidar::connect(lidar_config);
                        },
                        config.lidar_reconnect_interval,
                        decoder_slot,
                        vista::platform::completion_for(read_result)));
                read_started = true;
            }
        } catch (...) {
            vista::platform::stop_and_join_noexcept(workers, stop, bus);
            throw;
        }

        std::cout << "Capture is running continuously. Press Ctrl+C to stop.\n";
        try {
            vista::platform::wait_for_shutdown_request(stop);
        } catch (...) {
            vista::platform::stop_and_join_noexcept(workers, stop, bus);
            throw;
        }

        // Closing the bus wakes subscribers so they can drain queued messages,
        // finish their files, and exit cleanly.
        stop.request_stop();
        bus.close();

        std::vector<std::string> failures;
        vista::platform::join_workers(workers, stop, bus, failures);

        vista::platform::collect_worker_error(
            failures,
            runtime.threads.lidar_read.thread.name,
            read_result,
            read_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.lidar_decode.thread.name,
            decode_result,
            decode_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.raw_logger.thread.name,
            raw_logger_result,
            raw_logger_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.preprocessing.thread.name,
            preprocessing_result,
            preprocessing_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.pcd_logger.thread.name,
            pcd_logger_result,
            pcd_logger_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.grafana_bridge.thread.name,
            grafana_result,
            grafana_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.system_monitor.thread.name,
            system_monitor_result,
            system_monitor_started);
        vista::platform::collect_worker_error(
            failures,
            runtime.threads.pointcloud_websocket.thread.name,
            pointcloud_websocket_result,
            pointcloud_websocket_started);
        vista::platform::throw_if_worker_failures(failures);

        const auto packet_count =
            read_result->report ? read_result->report->message_count : 0;
        const auto point_count = preprocessing_result->report
                                     ? preprocessing_result->report->point_count
                                     : 0;
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started);

        std::cout << "Capture complete: " << packet_count << " packet(s), "
                  << point_count << " processed point(s), "
                  << static_cast<double>(elapsed.count()) / 1000.0
                  << " seconds\n";
        std::cout << "Ring-buffer drops: decoder="
                  << (decode_result->report
                          ? decode_result->report->dropped_message_count
                          : 0)
                  << ", raw-logger="
                  << (raw_logger_result->report
                          ? raw_logger_result->report->dropped_message_count
                          : 0)
                  << ", preprocessing="
                  << (preprocessing_result->report
                          ? preprocessing_result->report->dropped_message_count
                          : 0)
                  << ", pcd-logger="
                  << (pcd_logger_result->report
                          ? pcd_logger_result->report->dropped_message_count
                          : 0)
                  << '\n';
        if (grafana_result->report) {
            std::cout << "Grafana Live: published="
                      << grafana_result->report->published_measurements
                      << ", imu-messages="
                      << grafana_result->report->received_imu_messages
                      << ", failed-attempts="
                      << grafana_result->report->failed_publish_attempts
                      << ", input-drops="
                      << grafana_result->report->dropped_input_messages << '\n';
        }
        if (pointcloud_websocket_result->report) {
            std::cout
                << "Point-cloud WebSocket: frames="
                << pointcloud_websocket_result->report->broadcast_frames
                << ", client-deliveries="
                << pointcloud_websocket_result->report->client_deliveries
                << ", input-drops="
                << pointcloud_websocket_result->report->dropped_input_messages
                << ", server-failures="
                << pointcloud_websocket_result->report->server_failures
                << '\n';
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
