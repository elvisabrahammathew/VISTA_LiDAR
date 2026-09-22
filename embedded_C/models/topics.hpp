#pragma once

namespace vista::models::topics {

// LiDAR acquisition and point-cloud processing topics.
inline constexpr const char* lidar_raw = "lidar/raw";
inline constexpr const char* lidar_imu = "lidar/imu";
inline constexpr const char* pointcloud_decoded = "pointcloud/decoded";
inline constexpr const char* pointcloud_processed = "pointcloud/processed";

// Platform and application monitoring topics consumed by grafana_bridge.
inline constexpr const char* system_health = "monitor/system_health";
inline constexpr const char* worker_health = "monitor/worker_health";
inline constexpr const char* storage_health = "monitor/storage_health";
inline constexpr const char* power_thermal = "monitor/power_thermal";

}  // namespace vista::models::topics
