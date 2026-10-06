#pragma once

namespace vista::models::topics {

// LiDAR acquisition and point-cloud processing topics.
inline constexpr const char* lidar_raw = "lidar/raw";
inline constexpr const char* lidar_imu = "lidar/imu";
inline constexpr const char* pointcloud_decoded = "pointcloud/decoded";
// Sensor XYZ, floor intact: never gated by localization or transformed by mounting.
inline constexpr const char* pointcloud_cleaned_sensor = "pointcloud/cleaned_sensor";
// Motion-compensated localization outputs. Any active IMU uses lidar/imu above.
inline constexpr const char* pointcloud_world = "localization/pointcloud_world";
inline constexpr const char* localization_status = "localization/status";
// World-coordinate cloud including the floor, used by mapping and ground processing.
inline constexpr const char* pointcloud_cleaned = "pointcloud/cleaned";
inline constexpr const char* pointcloud_processed = "pointcloud/processed";
inline constexpr const char* ground_status = "processing/ground_status";
// Independent mapping types: RoomMapMessage, RoomMapViewMessage, RoomMapStatus.
// GroundStatusMessage and LocalizationStatus also live in models/mapping.
inline constexpr const char* room_map = "mapping/room_map";
inline constexpr const char* room_map_view = "mapping/room_map_view";
inline constexpr const char* room_map_status = "mapping/room_map_status";

// Platform and application monitoring topics consumed by grafana_bridge.
inline constexpr const char* system_health = "monitor/system_health";
inline constexpr const char* worker_health = "monitor/worker_health";
inline constexpr const char* storage_health = "monitor/storage_health";
inline constexpr const char* power_thermal = "monitor/power_thermal";

}  // namespace vista::models::topics
