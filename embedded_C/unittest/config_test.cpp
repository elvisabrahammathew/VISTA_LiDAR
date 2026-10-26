#include <chrono>
#include <ctime>
#include "1_Platform/compat/filesystem.hpp"
#include <string>
#include <atomic>
#include <thread>

#include "config.hpp"
#include "1_Platform/os/paths.hpp"
#include "unittest/test.hpp"

namespace {

const char* quanergy_config = R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 7
SensorIP(quanergy-m8, unitree-l2): 192.168.1.20
SensorPort(quanergy-m8, unitree-l2): 5000
Radar: None
)";

struct ConfigTestDirectory {
    vista::fs::path original = vista::fs::current_path();
    vista::fs::path root = vista::fs::temp_directory_path() /
        ("vista-config-test-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    ConfigTestDirectory() { vista::fs::create_directories(root); }
    ~ConfigTestDirectory() {
        std::error_code error;
        vista::fs::current_path(original, error);
        vista::fs::remove_all(root, error);
    }
};

std::string read_config_test_file(const vista::fs::path& file) {
    vista::io::ifstream input(file, std::ios::binary);
    VISTA_CHECK(input.is_open());
    std::ostringstream contents;
    contents << input.rdbuf();
    VISTA_CHECK(!input.bad());
    return contents.str();
}

}  // namespace

VISTA_TEST(config_bootstrap_creates_valid_defaults_and_blank_token_in_unicode_directory) {
    ConfigTestDirectory tree;
    const auto folder = tree.root / vista::fs::u8path(u8"first run space \u00e9");
    vista::fs::create_directory(folder);
    const auto created = vista::create_missing_startup_config_files(folder);
    VISTA_CHECK(created.device_config_created && created.grafana_secret_created);
    const auto config = vista::AppConfig::load(folder / "DeviceConfig.txt");
    (void)config.runtime_config();
#ifndef _WIN32
    VISTA_CHECK(config.unitree_serial_port == "/dev/ttyACM0");
#endif
    const auto secret = read_config_test_file(folder / "GrafanaSecret.txt");
    VISTA_CHECK(secret.find("GrafanaToken: \n") != std::string::npos);
    VISTA_CHECK(secret.find("glsa_") == std::string::npos);
    const auto repeated = vista::create_missing_startup_config_files(folder);
    VISTA_CHECK(!repeated.device_config_created && !repeated.grafana_secret_created);
}

VISTA_TEST(config_bootstrap_never_overwrites_existing_files_even_invalid_or_empty) {
    ConfigTestDirectory tree;
    const std::string device = "\xEF\xBB\xBF# user settings\r\nLidar: invalid\r\n";
    { vista::io::ofstream out(tree.root / "DeviceConfig.txt", std::ios::binary); out << device; }
    { vista::io::ofstream out(tree.root / "GrafanaSecret.txt"); }
    const auto created = vista::create_missing_startup_config_files(tree.root);
    VISTA_CHECK(!created.device_config_created && !created.grafana_secret_created);
    VISTA_CHECK(read_config_test_file(tree.root / "DeviceConfig.txt") == device);
    VISTA_CHECK(read_config_test_file(tree.root / "GrafanaSecret.txt").empty());
}

VISTA_TEST(config_bootstrap_missing_secret_preserves_custom_device_settings) {
    ConfigTestDirectory tree;
    { vista::io::ofstream out(tree.root / "DeviceConfig.txt", std::ios::binary); out << quanergy_config; }
    const auto created = vista::create_missing_startup_config_files(tree.root);
    VISTA_CHECK(!created.device_config_created && created.grafana_secret_created);
    VISTA_CHECK(read_config_test_file(tree.root / "DeviceConfig.txt") == quanergy_config);
    VISTA_CHECK(vista::AppConfig::load(tree.root / "DeviceConfig.txt").sensor_ip == "192.168.1.20");
}

VISTA_TEST(config_bootstrap_missing_device_preserves_existing_secret_exactly) {
    ConfigTestDirectory tree;
    const std::string secret = "# retained fixture\r\nGrafanaToken: test-token-not-a-real-credential\r\n";
    { vista::io::ofstream out(tree.root / "GrafanaSecret.txt", std::ios::binary); out << secret; }
    const auto created = vista::create_missing_startup_config_files(tree.root);
    VISTA_CHECK(created.device_config_created && !created.grafana_secret_created);
    VISTA_CHECK(read_config_test_file(tree.root / "GrafanaSecret.txt") == secret);
}

VISTA_TEST(config_bootstrap_reports_unusable_destination_without_replacing_it) {
    ConfigTestDirectory tree;
    vista::fs::create_directory(tree.root / "DeviceConfig.txt");
    VISTA_CHECK_THROWS(vista::create_missing_startup_config_files(tree.root));
    VISTA_CHECK(vista::fs::is_directory(tree.root / "DeviceConfig.txt"));
    VISTA_CHECK_THROWS(vista::create_missing_startup_config_files(tree.root / "absent"));
    VISTA_CHECK(!vista::fs::exists(tree.root / "absent"));
}

VISTA_TEST(config_bootstrap_exclusive_creation_has_one_winner_and_preserves_complete_text) {
    ConfigTestDirectory tree;
    const auto file = tree.root / "shared.txt";
    const std::string text(20000, 'x');
    std::atomic<int> winners{0}, errors{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < 8; ++i) {
        threads.emplace_back([&] {
            try { if (vista::platform::create_text_file_if_missing(file, text)) ++winners; }
            catch (...) { ++errors; }
        });
    }
    for (auto& thread : threads) thread.join();
    VISTA_CHECK(winners == 1 && errors == 0);
    VISTA_CHECK(read_config_test_file(file) == text);
}

VISTA_TEST(config_sidecar_default_path_ignores_working_directory) {
    ConfigTestDirectory tree;
    const auto expected = vista::platform::executable_path().parent_path() /
        "DeviceConfig.txt";
    vista::fs::current_path(tree.root);
    // A config in CWD must not override the executable's sidecar.
    { vista::io::ofstream out("DeviceConfig.txt"); out << "Lidar: invalid\n"; }
    VISTA_CHECK(vista::default_device_config_file() == expected);
    VISTA_CHECK(vista::default_device_config_file().is_absolute());
    VISTA_CHECK(vista::fs::current_path() == tree.root);
}

VISTA_TEST(config_sidecar_explicit_load_supports_unicode_and_relative_paths) {
    ConfigTestDirectory tree;
    const auto folder = tree.root / vista::fs::u8path(u8"settings space \u00e9");
    vista::fs::create_directory(folder);
    const auto file = folder / "DeviceConfig.txt";
    { vista::io::ofstream out(file); out << quanergy_config; }
    vista::fs::current_path(tree.root);
    const auto absolute = vista::AppConfig::load(file);
    const auto relative = vista::AppConfig::load(
        vista::fs::u8path(u8"settings space \u00e9") / "DeviceConfig.txt");
    VISTA_CHECK(absolute.sensor_ip == "192.168.1.20");
    VISTA_CHECK(relative.sensor_ip == absolute.sensor_ip);
    VISTA_CHECK(relative.lidar_type == absolute.lidar_type);
    VISTA_CHECK(vista::fs::current_path() == tree.root);
}

VISTA_TEST(config_sidecar_load_errors_identify_full_requested_path) {
    ConfigTestDirectory tree;
    const auto file = tree.root / "DeviceConfig.txt";
    for (const bool create_invalid : {false, true}) {
        if (create_invalid) {
            vista::io::ofstream out(file); out << "Lidar: invalid\n";
        }
        bool caught = false;
        try { (void)vista::AppConfig::load(file); }
        catch (const std::exception& error) {
            caught = true;
            VISTA_CHECK(std::string(error.what()).find(file.u8string()) !=
                std::string::npos);
        }
        VISTA_CHECK(caught);
    }
}

VISTA_TEST(config_lio_lightweight_robustness_switches_are_independent_and_default_on) {
    auto defaults=vista::parse_device_config_text(quanergy_config).lio;
    VISTA_CHECK(defaults.robust_imu_enabled && defaults.stationary_update_enabled);
    auto c=vista::parse_device_config_text(std::string(quanergy_config)+"LioRobustImuEnabled: 0\nLioStationaryUpdateEnabled: 1\n").lio;
    VISTA_CHECK(!c.robust_imu_enabled && c.stationary_update_enabled);
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"LioRobustImuEnabled: invalid\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"LioStationaryUpdateEnabled: invalid\n"));
}

VISTA_TEST(config_parses_upstream_imu_covariances_and_preserves_old_file_defaults) {
    const auto defaults=vista::parse_device_config_text(quanergy_config).lio;
    VISTA_CHECK_NEAR(defaults.imu_acceleration_covariance,.1,1e-12);
    VISTA_CHECK_NEAR(defaults.imu_gyroscope_covariance,.1,1e-12);
    VISTA_CHECK_NEAR(defaults.imu_acceleration_bias_covariance,.0001,1e-12);
    VISTA_CHECK_NEAR(defaults.imu_gyroscope_bias_covariance,.0001,1e-12);
    const auto c=vista::parse_device_config_text(std::string(quanergy_config)+
        "\nLioImuAccelerationCovariance: 0.2\nLioImuGyroscopeCovariance: 0.3\n"
        "LioImuAccelerationBiasCovariance: 0.004\nLioImuGyroscopeBiasCovariance: 0.005\n").lio;
    // TXT floating values use the parser's existing float representation.
    VISTA_CHECK_NEAR(c.imu_acceleration_covariance,.2,1e-7);
    VISTA_CHECK_NEAR(c.imu_gyroscope_covariance,.3,1e-7);
    VISTA_CHECK_NEAR(c.imu_acceleration_bias_covariance,.004,1e-8);
    VISTA_CHECK_NEAR(c.imu_gyroscope_bias_covariance,.005,1e-8);
    for(const auto* key:{"LioImuAccelerationCovariance","LioImuGyroscopeCovariance",
                         "LioImuAccelerationBiasCovariance","LioImuGyroscopeBiasCovariance"})
        for(const auto* value:{"0","-0.1","nan","inf"})
            VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\n"+key+": "+value+"\n"));
}

VISTA_TEST(config_sets_grafana_device_identity_and_room_map_performance_defaults) {
    const auto quanergy=vista::parse_device_config_text(quanergy_config);
    VISTA_CHECK(quanergy.grafana.selected_lidar_id=="quanergy-m8");
    VISTA_CHECK(quanergy.room_map.cache_max_tiles==128);
    VISTA_CHECK(quanergy.room_map_websocket.maximum_points==100000);
    const auto unitree=vista::parse_device_config_text(
        "Lidar: unitree-l2\nConnectionMode(unitree-l2): serial\nSerialPort(unitree-l2): COM4\nBaudRate(unitree-l2): 4000000\n");
    VISTA_CHECK(unitree.grafana.selected_lidar_id=="unitree-l2");
}

VISTA_TEST(config_point_lio_output_noise_units_saturation_and_matching_limits) {
    const auto defaults=vista::parse_device_config_text(quanergy_config).lio;
    VISTA_CHECK(defaults.velocity_process_covariance==20);
    VISTA_CHECK_NEAR(defaults.scan_voxel_m,.4,1e-12);
    VISTA_CHECK_NEAR(defaults.map_voxel_m,.4,1e-12);
    VISTA_CHECK(defaults.acceleration_process_covariance==500);
    VISTA_CHECK(defaults.angular_velocity_process_covariance==1000);
    const auto c=vista::parse_device_config_text(std::string(quanergy_config)+R"(
LioVelocityProcessCovariance: 12
LioAccelerationProcessCovariance: 300
LioAngularVelocityProcessCovariance: 800
LioLidarMeasurementCovariance: 0.02
LioImuAccelerationNorm: 1
LioImuAccelerationSaturation: 3
LioImuGyroscopeSaturation: 30
LioMinRangeMeters: 0.2
LioPlaneThresholdMeters: 0.08
)").lio;
    VISTA_CHECK(c.velocity_process_covariance==12 && c.acceleration_process_covariance==300 && c.angular_velocity_process_covariance==800);
    VISTA_CHECK_NEAR(c.lidar_measurement_covariance,.02,1e-7);
    VISTA_CHECK(c.imu_acceleration_norm==1 && c.imu_acceleration_saturation==3 && c.imu_gyroscope_saturation==30);
    VISTA_CHECK_NEAR(c.minimum_range_m,.2,1e-7);VISTA_CHECK_NEAR(c.plane_threshold_m,.08,1e-7);
    for(const auto* key:{"LioVelocityProcessCovariance","LioAccelerationProcessCovariance","LioAngularVelocityProcessCovariance",
                         "LioLidarMeasurementCovariance","LioImuAccelerationNorm","LioImuAccelerationSaturation",
                         "LioImuGyroscopeSaturation","LioMinRangeMeters","LioPlaneThresholdMeters"})
        for(const auto* value:{"0","-1","nan","inf"})
            VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\n"+key+": "+value+"\n"));
}

VISTA_TEST(config_room_mapping_settings_and_independent_mounting) {
    const auto config=vista::parse_device_config_text(std::string(quanergy_config)+R"(
MountZ(lidar): 2.5
RoomMapEnabled: 1
RoomMapVoxelSizeMeters: 0.1
RoomMapCacheMaxVoxels: 20000
RoomMapMinObservations: 4
RoomMapFreezeAfterSeconds: 0
RoomMapLoadExisting: 1
RoomMapFile: room.pcd
RoomMapWebSocketPort: 8766
)");
    VISTA_CHECK(!config.runtime_config().preprocessing.ground_removal);
    VISTA_CHECK(config.mounting && config.mounting->mount_z_m==2.5F);
    VISTA_CHECK(config.room_map.enabled && config.room_map.load_existing);
    VISTA_CHECK(config.room_map.cache_max_voxels==20000);
    VISTA_CHECK(config.room_map_websocket.room_map_lod);
    VISTA_CHECK(config.room_map.minimum_observations==4);
    VISTA_CHECK(config.room_map.freeze_after.count()==0);
    VISTA_CHECK(config.room_map_websocket.retained_snapshot);
    VISTA_CHECK(config.runtime_config().threads.room_mapping.enabled);
    VISTA_CHECK_NEAR(config.lio.initial_position_m[2],2.5,1e-6);
}

VISTA_TEST(config_rejects_invalid_mapping_and_colliding_websocket_ports) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapVoxelSizeMeters: 0\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+
        "\nGrafanaEnabled: true\nRoomMapWebSocketPort: 8767\n"));
}

VISTA_TEST(config_rejects_removed_room_map_save_toggle) {
    // Building mode always saves; obsolete TXT toggles must not imply otherwise.
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapSaveEnabled: 0\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapSaveEnabled: 1\n"));
}

VISTA_TEST(config_rejects_removed_total_map_limit_and_accepts_ram_lod_settings) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapMaxVoxels: 250000\n"));
    const auto options=vista::parse_device_config_text(std::string(quanergy_config)+
        "\nRoomMapCacheMaxVoxels: 10000\nRoomMapCacheMaxTiles: 8\nRoomMapTileSizeMeters: 0.5\nRoomMapLodPointsPerNode: 64\n");
    VISTA_CHECK(options.room_map.cache_max_voxels==10000);
    VISTA_CHECK(options.room_map.cache_max_tiles==8);
    VISTA_CHECK(options.room_map.tile_size_m==0.5F);
    VISTA_CHECK(options.room_map.lod_points_per_node==64);
}

VISTA_TEST(config_grafana_master_controls_both_websockets_regardless_of_legacy_keys) {
    const auto enabled=vista::parse_device_config_text(std::string(quanergy_config)+
        "\nPointCloudWebSocketEnabled: 0\nRoomMapWebSocketEnabled: 0\nGrafanaEnabled: true\nRoomMapEnabled: 0\n");
    VISTA_CHECK(!enabled.pointcloud_websocket.enabled && enabled.room_map_websocket.enabled);
    const auto runtime=enabled.runtime_config();
    VISTA_CHECK(runtime.threads.grafana_bridge.enabled && !runtime.threads.pointcloud_websocket.enabled);
    VISTA_CHECK(runtime.threads.room_map_websocket.enabled); // Opens even without a map.
    const auto disabled=vista::parse_device_config_text(std::string(quanergy_config)+
        "\nGrafanaEnabled: false\nPointCloudWebSocketEnabled: 1\nRoomMapWebSocketEnabled: 1\nRoomMapEnabled: 1\n");
    VISTA_CHECK(!disabled.pointcloud_websocket.enabled && !disabled.room_map_websocket.enabled);
    VISTA_CHECK(!disabled.runtime_config().threads.grafana_bridge.enabled);
    VISTA_CHECK(!disabled.runtime_config().threads.room_map_websocket.enabled);
    VISTA_CHECK(disabled.runtime_config().threads.room_mapping.enabled);
    VISTA_CHECK(disabled.runtime_config().threads.lidar_read.enabled);
}

VISTA_TEST(config_parses_ray_cleanup_limits_and_rejects_removed_control_file) {
    const auto config=vista::parse_device_config_text(std::string(quanergy_config)+R"(
RoomMapFreeSpaceMinObservations: 7
RoomMapRaycastMaxRaysPerWindow: 512
RoomMapRaycastMaxRangeMeters: 20
RoomMapRaycastSurfaceMarginMeters: 0.15
)");
    VISTA_CHECK(config.room_map.free_space_minimum_observations==7);
    VISTA_CHECK(config.room_map.raycast_maximum_rays_per_window==512);
    VISTA_CHECK_NEAR(config.room_map.raycast_maximum_range_m,20.0F,1.0e-6F);
    VISTA_CHECK_NEAR(config.room_map.raycast_surface_margin_m,0.15F,1.0e-6F);
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapFreeSpaceMinObservations: 1\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapControlFile: RoomMapControl.txt\n"));
}

VISTA_TEST(config_file_parses_quanergy_network_settings_and_notes) {
    const auto config = vista::parse_device_config_text(quanergy_config);

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::quanergy_m8);
    VISTA_CHECK(config.sensor_ip == "192.168.1.20");
    VISTA_CHECK(config.sensor_port == 5000);
    VISTA_CHECK(config.lidar_reconnect_interval == std::chrono::seconds(7));
    VISTA_CHECK(!config.raw_logging_enabled);
    VISTA_CHECK(!config.pointcloud_logging_enabled);
}

VISTA_TEST(config_file_parses_independent_lidar_logging_switches) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
RawLoggingEnabled(Lidar): 1
PointCloudLoggingEnabled(Lidar): 0
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
)");

    VISTA_CHECK(config.raw_logging_enabled);
    VISTA_CHECK(!config.pointcloud_logging_enabled);
    const auto runtime = config.runtime_config();
    VISTA_CHECK(runtime.threads.raw_logger.enabled);
    VISTA_CHECK(!runtime.threads.pcd_logger.enabled);
}

VISTA_TEST(config_file_rejects_invalid_lidar_logging_switch) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergy-m8
RawLoggingEnabled(Lidar): 2
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
)") );
}

VISTA_TEST(config_file_parses_realsense_with_first_matching_device) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsensel515
UsbSerial(realsensel515):
DepthWidth(realsensel515): 1024
DepthHeight(realsensel515): 768
DepthFps(realsensel515): 30
Radar:
)");

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::realsense_l515);
    VISTA_CHECK(!config.usb_serial);
    VISTA_CHECK(config.depth_width == 1024);
    VISTA_CHECK(config.depth_height == 768);
    VISTA_CHECK(config.depth_fps == 30);
}

VISTA_TEST(config_file_parses_explicit_realsense_usb_serial) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsense-l515
UsbSerial(realsensel515): 123456789
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30
)");

    VISTA_CHECK(config.usb_serial == std::string("123456789"));
}

VISTA_TEST(config_file_parses_unitree_auto_serial_and_udp_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: unitree-l2
SensorIP(quanergy-m8, unitree-l2): 192.168.1.62
SensorPort(quanergy-m8, unitree-l2): 6101
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
)");

    VISTA_CHECK(config.lidar_type == vista::devices::LidarType::unitree_l2);
    VISTA_CHECK(config.sensor_ip == "192.168.1.62");
    VISTA_CHECK(config.sensor_port == 6101);
    VISTA_CHECK(
        config.unitree_connection_mode ==
        vista::devices::UnitreeConnectionMode::automatic);
    VISTA_CHECK(config.unitree_local_ip == "192.168.1.2");
    VISTA_CHECK(config.unitree_local_port == 6201);
    VISTA_CHECK(config.unitree_serial_port == "COM3");
    VISTA_CHECK(config.unitree_baud_rate == 4'000'000);

    const auto lidar = config.lidar_config();
    VISTA_CHECK(lidar.unitree_l2.has_value());
    VISTA_CHECK(lidar.unitree_l2->sensor_port == 6101);
    VISTA_CHECK(lidar.unitree_l2->local_port == 6201);
    VISTA_CHECK(lidar.unitree_l2->initialize_on_connect);
    VISTA_CHECK(lidar.unitree_l2->work_mode == 9);
    VISTA_CHECK(lidar.unitree_l2->initialization_state != nullptr);
}

VISTA_TEST(config_unitree_init_settings_and_passive_replay) {
    const auto config = vista::parse_device_config_text(
        "Lidar: unitree-l2\nConnectionMode: serial\nSerialPort: COM6\n"
        "BaudRate: 4000000\nInitializeOnConnect(unitree-l2): 0\nWorkMode(unitree-l2): 8\n");
    VISTA_CHECK(!config.lidar_config().unitree_l2->initialize_on_connect);
    VISTA_CHECK(config.lidar_config().unitree_l2->work_mode == 8);
}

VISTA_TEST(config_unitree_rejects_non_uart_reserved_and_invalid_init_values) {
    const std::string base = "Lidar: unitree-l2\nConnectionMode: serial\n"
                             "SerialPort: COM6\nBaudRate: 4000000\n";
    for (const auto* value : {"0", "1", "11", "25", "32", "41", "-1", "invalid"})
        VISTA_CHECK_THROWS(vista::parse_device_config_text(base + "WorkMode: " + value + "\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(base + "InitializeOnConnect: 2\n"));
}

VISTA_TEST(config_unitree_udp_init_requires_uart_destination_but_passive_udp_does_not) {
    const std::string base = "Lidar: unitree-l2\nConnectionMode: udp\n"
        "SensorIP: 192.168.1.62\nSensorPort: 6101\nLocalIP: 192.168.1.2\nLocalPort: 6201\n";
    VISTA_CHECK_THROWS(vista::parse_device_config_text(base));
    const auto passive = vista::parse_device_config_text(base + "InitializeOnConnect: 0\n");
    VISTA_CHECK(!passive.lidar_config().unitree_l2->initialize_on_connect);
    const auto active = vista::parse_device_config_text(base + "SerialPort: COM6\nBaudRate: 4000000\n");
    VISTA_CHECK(active.lidar_config().unitree_l2->initialize_on_connect);
}

VISTA_TEST(config_unitree_init_does_not_change_other_lidar_sources) {
    const auto config = vista::parse_device_config_text(
        std::string(quanergy_config) + "InitializeOnConnect(unitree-l2): 1\nWorkMode(unitree-l2): 9\n");
    VISTA_CHECK(!config.lidar_config().unitree_l2.has_value());
    VISTA_CHECK(config.lidar_config().lidar_type == vista::devices::LidarType::quanergy_m8);
}

VISTA_TEST(config_file_parses_mounting_independently_of_retired_ground_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: unitree-l2
SensorIP(quanergy-m8, unitree-l2): 192.168.1.62
SensorPort(quanergy-m8, unitree-l2): 6101
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
MountX(lidar): -1.25
MountY(lidar): 2.5
MountZ(lidar): 0.75
MountRollDeg(lidar): 1.5
MountPitchDeg(lidar): -2.5
MountYawDeg(lidar): 90.0
)");

    VISTA_CHECK(config.mounting.has_value());
    const auto& ground = *config.mounting;
    VISTA_CHECK_NEAR(ground.mount_x_m, -1.25F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_y_m, 2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_z_m, 0.75F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_roll_deg, 1.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_pitch_deg, -2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_yaw_deg, 90.0F, 1.0e-6F);
    VISTA_CHECK(!config.runtime_config().preprocessing.ground_removal);
    VISTA_CHECK_NEAR(config.lio.initial_position_m[0],-1.25,1e-6);
    VISTA_CHECK_NEAR(config.lio.initial_position_m[1],2.5,1e-6);
    VISTA_CHECK_NEAR(config.lio.initial_position_m[2],.75,1e-6);
    VISTA_CHECK_NEAR(config.lio.initial_rpy_deg[0],1.5,1e-6);
    VISTA_CHECK_NEAR(config.lio.initial_rpy_deg[1],-2.5,1e-6);
    VISTA_CHECK_NEAR(config.lio.initial_rpy_deg[2],90,1e-6);
}

VISTA_TEST(config_rejects_retired_ground_keys_with_a_migration_message) {
    for(const auto* key:{"GroundMode","UseImuForGround","FloorZ",
                         "GroundDistanceThreshold","GroundNormalToleranceDeg",
                         "GroundCalibrationFrames","GroundMinInlierRatio"}) {
        bool rejected=false;
        try { (void)vista::parse_device_config_text(std::string(quanergy_config)+"\n"+key+"(lidar): 1\n"); }
        catch(const std::invalid_argument& e) {
            rejected=std::string(e.what()).find("Ground processing is retired")!=std::string::npos;
        }
        VISTA_CHECK(rejected);
    }
    for(const auto* key:{"MountX","MountY","MountZ","MountRollDeg","MountPitchDeg","MountYawDeg"})
        VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\n"+key+": nan\n"));
}

VISTA_TEST(config_file_accepts_all_ground_modes) {
    VISTA_CHECK(
        vista::application::parse_ground_mode("static") ==
        vista::application::GroundMode::static_height);
    VISTA_CHECK(
        vista::application::parse_ground_mode("ransac") ==
        vista::application::GroundMode::ransac);
    VISTA_CHECK(
        vista::application::parse_ground_mode("hybrid") ==
        vista::application::GroundMode::hybrid);
    VISTA_CHECK_THROWS(vista::application::parse_ground_mode("automatic"));
}

VISTA_TEST(config_file_accepts_com_port_as_legacy_usb_serial_name) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: realsensel515
ComPort(realsensel515): 987654321
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30
)");

    VISTA_CHECK(config.usb_serial == std::string("987654321"));
}

VISTA_TEST(config_file_requires_lidar) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        "Radar: None\n"));
}

VISTA_TEST(config_file_requires_quanergy_network_settings) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        "Lidar: quanergym8\n"));
}

VISTA_TEST(config_file_rejects_unimplemented_radar) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergym8
Radar: example-radar
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
)"));
}

VISTA_TEST(config_file_rejects_removed_duration_setting) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(
        std::string(quanergy_config) + "DurationSeconds: 10\n"));
}

VISTA_TEST(config_file_rejects_zero_lidar_reconnect_interval) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 0
SensorIP(quanergym8): 192.168.1.20
TcpPort(quanergym8): 5000
)"));
}

VISTA_TEST(config_formats_log_timestamp_in_local_system_time) {
    std::tm local_time{};
    local_time.tm_year = 2026 - 1900;
    local_time.tm_mon = 9 - 1;
    local_time.tm_mday = 16;
    local_time.tm_hour = 13;
    local_time.tm_min = 14;
    local_time.tm_sec = 5;
    local_time.tm_isdst = -1;
    const auto raw_time = std::mktime(&local_time);
    VISTA_CHECK(raw_time != static_cast<std::time_t>(-1));

    VISTA_CHECK(
        vista::format_log_timestamp(
            std::chrono::system_clock::from_time_t(raw_time)) ==
        "20260916_131405");
}

VISTA_TEST(config_building_generates_one_session_timestamp_for_all_outputs) {
    auto config=vista::parse_device_config_text(quanergy_config);
    config.room_map.enabled=true;
    config.room_map.file="an-older-map.pcd"; // Not an output override in build mode.
    config.raw_logging_enabled=true;
    config.pointcloud_logging_enabled=true;
    const auto start=std::chrono::system_clock::now();
    const auto stem=vista::format_log_timestamp(start);
    const vista::fs::path root="session-test-data";
    vista::configure_session_output_paths(config,root,start);
    const auto map_path=root/"maps"/("RoomMap_"+stem+".pcd");
    VISTA_CHECK(config.room_map.file==map_path);
    VISTA_CHECK(config.raw_path && *config.raw_path==root/"raw"/(stem+".bin"));
    VISTA_CHECK(config.pcd_path && *config.pcd_path==root/"processed"/(stem+".pcd"));
    vista::configure_session_output_paths(config,root,start);
    VISTA_CHECK(config.room_map.file==map_path); // Not regenerated on repeated setup.
}

VISTA_TEST(config_published_binary_uses_relocated_clone_data_root) {
    ConfigTestDirectory directory;
    const auto clone = directory.root / "clone with spaces";
    for (const auto* platform : {"windows-x64", "linux-x86_64", "linux-arm64"}) {
        const auto executable = clone / "publish" / platform / "vista_edge";
        VISTA_CHECK(vista::data_root_for_executable(executable,
            "old-build-machine/data") == clone / "data");
    }
    const auto relocated = directory.root / "another clone";
    VISTA_CHECK(vista::data_root_for_executable(relocated / "publish" /
        "linux-arm64" / "vista_edge", "old-build-machine/data") == relocated / "data");
}

VISTA_TEST(config_normal_build_keeps_existing_data_root) {
    VISTA_CHECK(vista::data_root_for_executable(
        "repo/embedded_C/out/build/windows-vs2019/Debug/vista_edge.exe",
        "repo/data") == vista::fs::path("repo/data"));
    VISTA_CHECK(vista::data_root_for_executable("repo/publish/not-a-platform/vista_edge",
        "repo/data") == vista::fs::path("repo/data"));
}

VISTA_TEST(config_loading_keeps_explicit_input_and_does_not_guess_empty_input) {
    auto config=vista::parse_device_config_text(quanergy_config);
    config.room_map.load_existing=true;
    const vista::fs::path reference="maps/RoomMap_20260916_131405.pcd";
    config.room_map.file=reference;
    vista::configure_session_output_paths(config,"session-test-data",std::chrono::system_clock::now());
    VISTA_CHECK(config.room_map.file==reference);
    VISTA_CHECK(!config.raw_path && !config.pcd_path);
    config.room_map.file.clear();
    vista::configure_session_output_paths(config,"session-test-data",std::chrono::system_clock::now());
    VISTA_CHECK(config.room_map.file.empty()); // Worker reports LOAD ERROR.
}

VISTA_TEST(config_exposes_independent_worker_enable_and_priority_settings) {
    const auto config = vista::parse_device_config_text(quanergy_config);
    const auto runtime = config.runtime_config();

    VISTA_CHECK(runtime.threads.lidar_read.enabled);
    VISTA_CHECK(runtime.threads.lidar_read.thread.priority.level() == 1);
    VISTA_CHECK(runtime.threads.lidar_decode.thread.priority.level() == 2);
    VISTA_CHECK(runtime.threads.preprocessing.thread.priority.level() == 3);
    VISTA_CHECK(runtime.threads.grafana_bridge.thread.priority.level() == 4);
    VISTA_CHECK(
        runtime.threads.pointcloud_websocket.thread.priority.level() == 4);
    VISTA_CHECK(runtime.threads.system_monitor.thread.priority.level() == 4);
    VISTA_CHECK(runtime.queues.raw_capacity == 32);
    VISTA_CHECK(runtime.queues.telemetry_capacity == 16);
}

VISTA_TEST(config_file_parses_pointcloud_websocket_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
GrafanaEnabled: true
PointCloudWebSocketBindAddress: 0.0.0.0
PointCloudWebSocketPort: 9876
PointCloudWebSocketMaxPoints: 75000
PointCloudWebSocketMaxClients: 3
PointCloudWebSocketPublishIntervalMilliseconds: 125
PointCloudWebSocketRetryIntervalSeconds: 4
)");

    VISTA_CHECK(!config.pointcloud_websocket.enabled); // Legacy values accepted, live 3D retired.
    VISTA_CHECK(config.pointcloud_websocket.bind_address == "0.0.0.0");
    VISTA_CHECK(config.pointcloud_websocket.port == 9876);
    VISTA_CHECK(config.pointcloud_websocket.maximum_points == 75'000);
    VISTA_CHECK(config.pointcloud_websocket.maximum_clients == 3);
    VISTA_CHECK(
        config.pointcloud_websocket.publish_interval ==
        std::chrono::milliseconds(125));
    VISTA_CHECK(
        config.pointcloud_websocket.retry_interval ==
        std::chrono::seconds(4));
    VISTA_CHECK(!config.runtime_config().threads.pointcloud_websocket.enabled);
}

VISTA_TEST(config_file_parses_grafana_live_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
GrafanaEnabled: yes
GrafanaHost: localhost
GrafanaPort: 3100
GrafanaNamespace: factory_floor
GrafanaPublishIntervalMilliseconds: 250
GrafanaRetryIntervalSeconds: 9
SystemMonitorIntervalMilliseconds: 1500
)");

    VISTA_CHECK(config.grafana.enabled);
    VISTA_CHECK(config.grafana.host == "localhost");
    VISTA_CHECK(config.grafana.port == 3100);
    VISTA_CHECK(config.grafana.name_space == "factory_floor");
    VISTA_CHECK(config.grafana.publish_interval == std::chrono::milliseconds(250));
    VISTA_CHECK(config.grafana.retry_interval == std::chrono::seconds(9));
    VISTA_CHECK(
        config.system_monitor.sample_interval == std::chrono::milliseconds(1500));
}

VISTA_TEST(config_file_rejects_invalid_grafana_namespace) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(R"(
Lidar: quanergy-m8
SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141
GrafanaNamespace: invalid/name
)") );
}

VISTA_TEST(config_room_map_checkpoint_and_websocket_share_one_interval) {
    const vista::AppConfig initial;
    VISTA_CHECK(initial.room_map_websocket.publish_interval==initial.room_map.publish_interval);
    const auto config=vista::parse_device_config_text(std::string(quanergy_config)+
        "RoomMapPublishIntervalMilliseconds: 275\n"
        "PointCloudWebSocketPublishIntervalMilliseconds: 75\n"
        "GrafanaPublishIntervalMilliseconds: 900\n");
    VISTA_CHECK(config.room_map.publish_interval==std::chrono::milliseconds(275));
    VISTA_CHECK(config.room_map_websocket.publish_interval==config.room_map.publish_interval);
    VISTA_CHECK(config.pointcloud_websocket.publish_interval==std::chrono::milliseconds(75));
    VISTA_CHECK(config.grafana.publish_interval==std::chrono::milliseconds(900));
    const auto defaults=vista::parse_device_config_text(quanergy_config);
    VISTA_CHECK(defaults.room_map_websocket.publish_interval==defaults.room_map.publish_interval);
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+
        "RoomMapPublishIntervalMilliseconds: 0\n"));
}
