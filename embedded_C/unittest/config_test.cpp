#include <chrono>
#include <ctime>
#include <filesystem>
#include <string>

#include "config.hpp"
#include "unittest/test.hpp"

namespace {

const char* quanergy_config = R"(
Lidar: quanergym8
ReconnectIntervalSeconds(Lidar): 7
SensorIP(quanergy-m8, unitree-l2): 192.168.1.20
SensorPort(quanergy-m8, unitree-l2): 5000
Radar: None
)";

}  // namespace

VISTA_TEST(config_sets_grafana_device_identity_and_room_map_performance_defaults) {
    const auto quanergy=vista::parse_device_config_text(quanergy_config);
    VISTA_CHECK(quanergy.grafana.selected_lidar_id=="quanergy-m8");
    VISTA_CHECK(quanergy.room_map.cache_max_tiles==128);
    VISTA_CHECK(quanergy.room_map_websocket.maximum_points==100000);
    const auto unitree=vista::parse_device_config_text(
        "Lidar: unitree-l2\nConnectionMode(unitree-l2): serial\nSerialPort(unitree-l2): COM4\nBaudRate(unitree-l2): 4000000\n");
    VISTA_CHECK(unitree.grafana.selected_lidar_id=="unitree-l2");
}

VISTA_TEST(config_room_mapping_settings_and_independent_mounting) {
    const auto config=vista::parse_device_config_text(std::string(quanergy_config)+R"(
GroundMode(lidar): none
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
    VISTA_CHECK(!config.ground_removal);
    VISTA_CHECK(config.mounting && config.mounting->mount_z_m==2.5F);
    VISTA_CHECK(config.room_map.enabled && config.room_map.load_existing);
    VISTA_CHECK(config.room_map.cache_max_voxels==20000);
    VISTA_CHECK(config.room_map_websocket.room_map_lod);
    VISTA_CHECK(config.room_map.minimum_observations==4);
    VISTA_CHECK(config.room_map.freeze_after.count()==0);
    VISTA_CHECK(config.room_map_websocket.retained_snapshot);
    VISTA_CHECK(config.runtime_config().threads.room_mapping.enabled);
    VISTA_CHECK(config.runtime_config().threads.ground_processing.enabled);
}

VISTA_TEST(config_rejects_invalid_mapping_and_colliding_websocket_ports) {
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+"\nRoomMapVoxelSizeMeters: 0\n"));
    VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string(quanergy_config)+
        "\nGrafanaEnabled: true\nRoomMapWebSocketPort: 8765\n"));
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
    VISTA_CHECK(enabled.pointcloud_websocket.enabled && enabled.room_map_websocket.enabled);
    const auto runtime=enabled.runtime_config();
    VISTA_CHECK(runtime.threads.grafana_bridge.enabled && runtime.threads.pointcloud_websocket.enabled);
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
}

VISTA_TEST(config_file_parses_mounting_ground_and_imu_settings) {
    const auto config = vista::parse_device_config_text(R"(
Lidar: unitree-l2
SensorIP(quanergy-m8, unitree-l2): 192.168.1.62
SensorPort(quanergy-m8, unitree-l2): 6101
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
GroundMode(lidar): hybrid
UseImuForGround(lidar): 1
MountX(lidar): -1.25
MountY(lidar): 2.5
MountZ(lidar): 0.75
MountRollDeg(lidar): 1.5
MountPitchDeg(lidar): -2.5
MountYawDeg(lidar): 90.0
FloorZ(lidar): 0.0
GroundDistanceThreshold(lidar): 0.10
GroundNormalToleranceDeg(lidar): 15.0
GroundCalibrationFrames(lidar): 50
GroundMinInlierRatio(lidar): 0.20
)");

    VISTA_CHECK(config.ground_removal.has_value());
    const auto& ground = *config.ground_removal;
    VISTA_CHECK(ground.mode == vista::application::GroundMode::hybrid);
    VISTA_CHECK(ground.use_imu);
    VISTA_CHECK_NEAR(ground.mount_x_m, -1.25F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_y_m, 2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_z_m, 0.75F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_roll_deg, 1.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_pitch_deg, -2.5F, 1.0e-6F);
    VISTA_CHECK_NEAR(ground.mount_yaw_deg, 90.0F, 1.0e-6F);
    VISTA_CHECK(ground.calibration_frames == 50);
    VISTA_CHECK_NEAR(ground.minimum_inlier_ratio, 0.20F, 1.0e-6F);
    VISTA_CHECK(config.runtime_config().preprocessing.ground_removal.has_value());
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
    const std::filesystem::path root="session-test-data";
    vista::configure_session_output_paths(config,root,start);
    const auto map_path=root/"maps"/("RoomMap_"+stem+".pcd");
    VISTA_CHECK(config.room_map.file==map_path);
    VISTA_CHECK(config.raw_path && *config.raw_path==root/"raw"/(stem+".bin"));
    VISTA_CHECK(config.pcd_path && *config.pcd_path==root/"processed"/(stem+".pcd"));
    vista::configure_session_output_paths(config,root,start);
    VISTA_CHECK(config.room_map.file==map_path); // Not regenerated on repeated setup.
}

VISTA_TEST(config_loading_keeps_explicit_input_and_does_not_guess_empty_input) {
    auto config=vista::parse_device_config_text(quanergy_config);
    config.room_map.load_existing=true;
    const std::filesystem::path reference="maps/RoomMap_20260916_131405.pcd";
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

    VISTA_CHECK(config.pointcloud_websocket.enabled);
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
    VISTA_CHECK(config.runtime_config().threads.pointcloud_websocket.enabled);
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
