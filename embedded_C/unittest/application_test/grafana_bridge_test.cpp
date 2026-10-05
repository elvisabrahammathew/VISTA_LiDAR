#include <string>
#include <set>

#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "unittest/test.hpp"

VISTA_TEST(grafana_bridge_formats_pointcloud_as_influx_line_protocol) {
    const vista::application::PointCloudTelemetry telemetry{
        "quanergy m8,roof",
        1'725'000'000'000'000'000ULL,
        12'000,
        3'250,
        2,
        19.5,
        4.25,
        true,
        true,
        -10.0F,
        12.0F,
        -4.0F,
        5.0F,
        -2.5F,
        1.5F,
    };

    const auto line =
        vista::application::format_pointcloud_measurement(telemetry);

    VISTA_CHECK(line.find("pointcloud,lidar_id=quanergy\\ m8\\,roof") == 0);
    VISTA_CHECK(line.find("online=1i") != std::string::npos);
    VISTA_CHECK(line.find("input_point_count=12000i") != std::string::npos);
    VISTA_CHECK(line.find("point_count=3250i") != std::string::npos);
    VISTA_CHECK(line.find("min_z_m=-2.5") != std::string::npos);
    VISTA_CHECK(
        line.find(" 1725000000000000000") != std::string::npos);
}

VISTA_TEST(grafana_bridge_formats_imu_as_influx_line_protocol) {
    vista::application::ImuTelemetry telemetry;
    telemetry.lidar_id = "unitree-l2";
    telemetry.timestamp_ns = 1'725'000'000'000'000'001ULL;
    telemetry.sample_rate_hz = 200.0;
    telemetry.online = true;
    telemetry.sample.orientation_x = 0.1F;
    telemetry.sample.orientation_w = 0.9F;
    telemetry.sample.angular_velocity_z_rad_s = 1.3F;
    telemetry.sample.linear_acceleration_x_m_s2 = 9.7F;

    const auto line = vista::application::format_imu_measurement(telemetry);

    VISTA_CHECK(line.find("imu,lidar_id=unitree-l2") == 0);
    VISTA_CHECK(line.find("online=1i") != std::string::npos);
    VISTA_CHECK(line.find("sample_rate_hz=200") != std::string::npos);
    VISTA_CHECK(line.find("orientation_x=") != std::string::npos);
    VISTA_CHECK(line.find("angular_velocity_z_rad_s=") != std::string::npos);
    VISTA_CHECK(line.find("linear_acceleration_x_m_s2=") != std::string::npos);
}

VISTA_TEST(grafana_bridge_formats_ground_status_as_influx_line_protocol) {
    vista::models::GroundStatus ground;
    ground.configured_mode="hybrid";
    ground.state=vista::models::GroundState::valid;
    ground.calibrated=true;
    ground.plane_c=1.0F;
    ground.ground_tilt_deg=1.5F;
    ground.ground_inlier_ratio=0.72F;
    ground.rms_residual_m=0.018F;
    ground.removed_ground_ratio=0.61F;
    vista::models::LidarGroundStatusMessage message(
        "unitree-l2",1,std::nullopt,2,std::move(ground));

    const auto line=vista::application::format_ground_measurement(message);
    VISTA_CHECK(line.find("ground_status,lidar_id=unitree-l2,mode=hybrid ground_tilt_deg=")==0);
    VISTA_CHECK(line.find(",state=")==std::string::npos);
    VISTA_CHECK(line.find("state_code=")==std::string::npos); // Dedicated ground_state channel.
    VISTA_CHECK(line.find("ground_inlier_ratio=0.72")!=std::string::npos);
    VISTA_CHECK(line.find("rms_residual_m=")!=std::string::npos);

    const auto state_line=
        vista::application::format_ground_state_measurement(message);
    VISTA_CHECK(state_line.find("ground_state state_code=1i ")==0);
    VISTA_CHECK(state_line.find(',')==std::string::npos);
}

VISTA_TEST(grafana_bridge_rejects_invalid_channel_namespace) {
    auto config = vista::application::GrafanaBridgeConfig{};
    config.name_space = "factory/line";
    VISTA_CHECK_THROWS(
        vista::application::validate_grafana_bridge_config(config));
}

VISTA_TEST(grafana_bridge_formats_service_account_authorization) {
    VISTA_CHECK(
        vista::application::make_grafana_bearer_authorization("glsa_test-token") ==
        "Bearer glsa_test-token");
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization(""));
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization("glsa_bad token"));
    VISTA_CHECK_THROWS(
        vista::application::make_grafana_bearer_authorization("token-name-only"));
}
VISTA_TEST(grafana_bridge_formats_mapping_status_without_labels) {
    vista::models::RoomMapStatus status;
    status.timestamp_ns=123456;
    status.state=vista::models::RoomMapState::frozen;
    status.point_count=1200;
    status.cache_max_voxels=250000;
    status.active_build_seconds=30;
    status.cleared_voxels=7;
    status.raycasts=128;
    status.last_integration_ms=2.5;status.tile_writes=9;status.view_queries=12;
    const auto line=vista::application::format_room_map_measurement(status);
    VISTA_CHECK(line.find("room_map_status state_code=2i,point_count=1200i")==0);
    VISTA_CHECK(line.find("cache_max_voxels=250000i")!=std::string::npos);
    VISTA_CHECK(line.find("cache_voxels=0i")!=std::string::npos);
    VISTA_CHECK(line.find("disk_tiles=0i")!=std::string::npos);
    VISTA_CHECK(line.find("cleared_voxels=7i")!=std::string::npos);
    VISTA_CHECK(line.find("raycasts=128i")!=std::string::npos);
    VISTA_CHECK(line.find("last_integration_ms=2.5")!=std::string::npos);
    VISTA_CHECK(line.find("tile_writes=9i")!=std::string::npos);
    VISTA_CHECK(line.find("view_queries=12i")!=std::string::npos);
    VISTA_CHECK(line.find(" 123456")!=std::string::npos);
}

VISTA_TEST(grafana_current_lidar_uses_one_tag_free_channel_for_online_and_offline) {
    auto value=vista::application::make_offline_pointcloud_telemetry("unitree-l2");
    VISTA_CHECK(value.lidar_id=="unitree-l2" && !value.online);
    const auto offline=vista::application::format_current_pointcloud_measurement(value);
    VISTA_CHECK(offline.find("pointcloud_current online=0i")==0);
    VISTA_CHECK(offline.find("lidar_id=")==std::string::npos);
    value.online=true;value.processing_ms=650;value.dropped_messages=9;value.frames_per_second=120;
    const auto online=vista::application::format_current_pointcloud_measurement(value);
    VISTA_CHECK(online.find("pointcloud_current online=1i")==0);
    VISTA_CHECK(online.find("data_age_ms=650")!=std::string::npos);
    VISTA_CHECK(online.find("bridge_dropped_messages=")==std::string::npos); // Pipeline History owns drops.
    VISTA_CHECK(online.find("message_rate_hz=120")!=std::string::npos);
    VISTA_CHECK_THROWS(vista::application::make_offline_pointcloud_telemetry(""));
}

namespace {
std::set<std::string> field_names(const std::string& line) {
    // Test fixtures use tags without spaces. Parse only the field set, not tags/time.
    const auto start=line.find(' ')+1;
    const auto end=line.find(' ',start);
    std::set<std::string> names;
    for(auto position=start;position<end;) {
        const auto equals=line.find('=',position);
        names.insert(line.substr(position,equals-position));
        const auto comma=line.find(',',equals);
        if(comma==std::string::npos || comma>=end) break;
        position=comma+1;
    }
    return names;
}
}

VISTA_TEST(grafana_compact_measurements_only_include_current_dashboard_fields) {
    vista::application::PointCloudTelemetry cloud;cloud.timestamp_ns=123;
    const std::set<std::string> cloud_fields{"online","input_point_count","point_count","data_age_ms","message_rate_hz"};
    VISTA_CHECK(field_names(vista::application::format_current_pointcloud_measurement(cloud))==cloud_fields);
    vista::models::GroundStatus ground;ground.configured_mode="hybrid";
    vista::models::LidarGroundStatusMessage message("unitree-l2",1,std::nullopt,2,std::move(ground));
    const std::set<std::string> ground_fields{"ground_tilt_deg","ground_height_error_m","ground_inlier_ratio","rms_residual_m","removed_ground_ratio"};
    VISTA_CHECK(field_names(vista::application::format_ground_measurement(message))==ground_fields);
    VISTA_CHECK(field_names(vista::application::format_ground_state_measurement(message))==std::set<std::string>{"state_code"});
    vista::models::RoomMapStatus map;map.timestamp_ns=123;
    const std::set<std::string> map_fields{
        "state_code","point_count","candidate_voxels","active_build_seconds","dropped_input_messages",
        "cleared_voxels","raycasts","free_space_checks","ray_budget_skipped_points","ray_traversal_steps","missing_origin_messages",
        "cache_voxels","cache_max_voxels","cache_tiles","cache_evictions","disk_tiles",
        "last_integration_ms","max_integration_ms","last_map_lock_wait_ms","last_checkpoint_ms","last_view_ms","last_view_lock_wait_ms",
        "integrated_frames","preview_queries","view_queries","cache_misses","tile_reads","tile_writes","lod_node_writes",
        "tile_read_total_ms","tile_write_total_ms","lod_update_total_ms"};
    VISTA_CHECK(field_names(vista::application::format_room_map_measurement(map))==map_fields);
}
