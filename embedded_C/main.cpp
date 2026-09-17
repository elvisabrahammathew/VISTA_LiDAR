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
        } else {
            std::cout << "LiDAR configuration: "
                      << vista::devices::to_string(config.lidar_type) << " at "
                      << config.sensor_ip;
            if (config.tcp_port) {
                std::cout << ':' << *config.tcp_port;
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
            vista::models::topics::pointcloud_decoded,
            runtime.queues.decoded_capacity);
        bus.configure_topic(
            vista::models::topics::pointcloud_processed,
            runtime.queues.processed_capacity);

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

        std::vector<vista::platform::WorkerHandle> workers;
        workers.reserve(5);
        bool read_started = false;
        bool decode_started = false;
        bool raw_logger_started = false;
        bool preprocessing_started = false;
        bool pcd_logger_started = false;

        try {
            // The user controls worker creation and startup order in main.
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
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
