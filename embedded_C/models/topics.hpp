#pragma once

namespace vista::models::topics {

// LiDAR acquisition and point-cloud processing topics.
inline constexpr const char* lidar_raw = "lidar/raw";
inline constexpr const char* pointcloud_decoded = "pointcloud/decoded";
inline constexpr const char* pointcloud_processed = "pointcloud/processed";

}  // namespace vista::models::topics
