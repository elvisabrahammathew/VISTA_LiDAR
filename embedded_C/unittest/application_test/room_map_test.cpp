#include <array>
#include <chrono>
#include "1_Platform/compat/filesystem.hpp"
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "2_Transport/storage/room_map_storage.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "4_Applications/mapping/room_map/room_map.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"

namespace {
using namespace std::chrono_literals;
vista::models::PointXYZIRT map_point(float x, float y, float z) { return {x,y,z,12,0,0,0}; }
vista::models::RoomMapFrame map_geometry(const vista::models::PointCloudFrame& cloud) {
    vista::models::RoomMapFrame geometry;geometry.timestamp_ns=cloud.timestamp_ns;
    for(const auto& p:cloud.points) geometry.points.push_back({p.x,p.y,p.z,p.intensity});
    return geometry;
}
vista::application::RoomMapConfig settings() {
    vista::application::RoomMapConfig value;
    value.enabled = true;
    value.voxel_size_m = 1.0F;
    value.cache_max_voxels = 10;
    value.tile_size_m = 1;
    value.minimum_observations = 2;
    value.freeze_after = 0ms;
    return value;
}
vista::models::PointCloudFrame ray_frame(std::initializer_list<vista::models::PointXYZIRT> points,
    std::array<float,3> origin = {0.25F,0.25F,0.25F}) {
    return {1,std::vector<vista::models::PointXYZIRT>(points),origin};
}
}

VISTA_TEST(room_map_confirms_distinct_time_windows_and_preserves_floor) {
    vista::application::RoomMapAccumulator map(settings());
    const vista::models::PointCloudFrame cloud(1,{map_point(1.1F,0,0),map_point(1.2F,0,0),map_point(2.1F,0,2)});
    map.integrate(cloud,0ms);
    map.integrate(cloud,20ms);  // Many points/packets in one window still count once.
    VISTA_CHECK(map.snapshot().points.empty());
    map.integrate(cloud,100ms);
    VISTA_CHECK(map.snapshot().points.size()==2);
    const auto output = map.snapshot();
    bool floor_found=false;
    for(const auto& point:output.points) floor_found=floor_found || point.z==0.0F;
    VISTA_CHECK(floor_found);
    map.integrate(vista::models::PointCloudFrame(2,{}),200ms);
    VISTA_CHECK(map.snapshot().points.size()==2);  // Missing points are not deleted.
}

VISTA_TEST(room_map_freezes_resumes_and_resets) {
    auto config=settings(); config.minimum_observations=1; config.freeze_after=200ms;
    vista::application::RoomMapAccumulator map(config);
    map.integrate({1,{map_point(1,0,0)}},0ms);
    map.integrate({2,{map_point(2,0,0)}},200ms);
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::frozen);
    const auto frozen=map.snapshot().points;
    VISTA_CHECK(!map.integrate({3,{map_point(3,0,0)}},300ms));
    VISTA_CHECK(map.snapshot().points==frozen);
    map.resume();
    map.integrate({4,{map_point(3,0,0)}},300ms);
    VISTA_CHECK(map.snapshot().points.size()==3);
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::building);
    map.reset();
    VISTA_CHECK(map.snapshot().points.empty());
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::empty);
}

VISTA_TEST(room_map_cache_is_bounded_without_rejecting_new_tiles) {
    auto config=settings(); config.cache_max_voxels=2;config.cache_max_tiles=2;
    vista::application::RoomMapAccumulator map(config);
    map.integrate({1,{map_point(1,0,0),map_point(2,0,0)}},0ms);
    map.integrate({2,{map_point(1,0,0),map_point(3,0,0)}},100ms);
    VISTA_CHECK(map.status().point_count==1);
    VISTA_CHECK(map.status().candidate_voxels==2);
    VISTA_CHECK(map.status().rejected_new_voxels==0);
    VISTA_CHECK(map.status().cache_voxels<=2);
    map.integrate({3,{map_point(3,0,0)}},12'000ms);
    map.integrate({4,{map_point(3,0,0)}},12'100ms);
    VISTA_CHECK(map.status().point_count==2);
    VISTA_CHECK(map.status().candidate_voxels<=1); // An inactive disk candidate is pruned when loaded again.
}

VISTA_TEST(room_map_confirmation_counter_stays_bounded_after_many_observations) {
    auto config=settings(); config.minimum_observations=100;
    vista::application::RoomMapAccumulator map(config);
    for (int index=0; index<150; ++index)
        map.integrate({1,{map_point(-0.1F,0,0)}},std::chrono::milliseconds(index*100));
    VISTA_CHECK(map.status().point_count==1);
    VISTA_CHECK(map.status().candidate_voxels==0);
}

VISTA_TEST(room_map_ignores_invalid_points_and_does_not_auto_freeze_empty_map) {
    auto config=settings(); config.freeze_after=100ms;
    vista::application::RoomMapAccumulator map(config);
    map.integrate({1,{map_point(std::numeric_limits<float>::quiet_NaN(),0,0)}},500ms);
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::empty);
    VISTA_CHECK(map.status().point_count==0);
}

VISTA_TEST(room_map_snapshot_roundtrip_and_replacement) {
    const auto path=vista::fs::temp_directory_path()/
        ("vista-room-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    const vista::models::PointCloudFrame cloud(0,{map_point(1.1234567F,0,0),map_point(2,3,4)});
    vista::transport::save_room_map_snapshot(path,map_geometry(cloud));
    const auto loaded=vista::transport::load_room_map_snapshot(path,10);
    VISTA_CHECK(loaded.points==map_geometry(cloud).points);
    VISTA_CHECK_THROWS(vista::transport::load_room_map_snapshot(path,1));
    const vista::models::PointCloudFrame smaller(0,{map_point(5,6,7)});
    vista::transport::save_room_map_snapshot(path,map_geometry(smaller));
    VISTA_CHECK(vista::transport::load_room_map_snapshot(path,10).points==map_geometry(smaller).points);
    vista::application::RoomMapAccumulator map(settings());
    map.restore(loaded);
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::loaded);
    VISTA_CHECK(!map.integrate(smaller,100ms));
    VISTA_CHECK(map.snapshot().points.size()==2);
    vista::fs::remove(path);
}

VISTA_TEST(room_map_session_file_claims_suffixes_and_updates_only_its_own_file) {
    const auto root=vista::fs::temp_directory_path()/
        ("vista-session-claims-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto preferred=root/"RoomMap_20260916_131405.pcd";
    const auto first_suffix=root/"RoomMap_20260916_131405_1.pcd";
    const auto second_suffix=root/"RoomMap_20260916_131405_2.pcd";
    const vista::models::PointCloudFrame original(1,{map_point(1,0,0)});
    const vista::models::PointCloudFrame first(2,{map_point(2,0,0)});
    const vista::models::PointCloudFrame second(3,{map_point(3,0,0)});
    vista::transport::save_room_map_snapshot(preferred,map_geometry(original));
    const auto original_stamp=vista::fs::last_write_time(preferred);
    {
        vista::transport::RoomMapSessionFile a(preferred),b(preferred);
        a.save(map_geometry(first)); b.save(map_geometry(second)); // Same preferred timestamp; exclusive claims differ.
        VISTA_CHECK(a.path()==first_suffix && b.path()==second_suffix);
        a.save(map_geometry(second));
        VISTA_CHECK(a.path()==first_suffix);
        VISTA_CHECK(vista::transport::load_room_map_snapshot(first_suffix,10).points==map_geometry(second).points);
        VISTA_CHECK(vista::transport::load_room_map_snapshot(second_suffix,10).points==map_geometry(second).points);
    }
    VISTA_CHECK(vista::fs::last_write_time(preferred)==original_stamp);
    VISTA_CHECK(vista::transport::load_room_map_snapshot(preferred,10).points==map_geometry(original).points);
    vista::fs::remove(first_suffix); vista::fs::remove(second_suffix);
    vista::fs::remove(preferred); vista::fs::remove(root); // Exact empty test directory.
}

VISTA_TEST(room_map_session_file_is_lazy_and_does_not_create_no_data_output) {
    const auto preferred=vista::fs::temp_directory_path()/
        ("vista-lazy-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    { vista::transport::RoomMapSessionFile session(preferred); }
    VISTA_CHECK(!vista::fs::exists(preferred));
}

VISTA_TEST(room_map_preserves_confirmed_wall_behind_a_person_who_stays) {
    vista::application::RoomMapAccumulator map(settings());
    const auto wall=ray_frame({map_point(6.2F,0.25F,0.25F)});
    const auto person=ray_frame({map_point(2.2F,0.25F,0.25F)});
    map.integrate(wall,0ms); map.integrate(wall,100ms);
    for(int window=2;window<20;++window) map.integrate(person,std::chrono::milliseconds(window*100));
    VISTA_CHECK(map.status().point_count==2); // Person AND previously confirmed wall.
    VISTA_CHECK(map.status().cleared_voxels==0);
    map.integrate({1,{}},10'000ms); // Missing observations are not free-space evidence.
    VISTA_CHECK(map.status().point_count==2);
}

VISTA_TEST(room_map_clears_a_departed_person_only_after_repeated_free_windows) {
    vista::application::RoomMapAccumulator map(settings());
    const auto person=ray_frame({map_point(2.2F,0.25F,0.25F)});
    const auto wall=ray_frame({map_point(6.2F,0.25F,0.25F)});
    map.integrate(person,0ms); map.integrate(person,100ms);
    map.integrate(wall,200ms);
    map.integrate(wall,250ms); // Many rays in one window count as ONE free observation.
    for(int window=3;window<7;++window) map.integrate(wall,std::chrono::milliseconds(window*100));
    VISTA_CHECK(map.status().point_count==2);
    map.integrate(wall,700ms); // Commits the fifth free window.
    VISTA_CHECK(map.status().point_count==1);
    VISTA_CHECK(map.status().cleared_voxels==1);
    VISTA_CHECK(map.status().free_space_checks>=5);
}

VISTA_TEST(room_map_hits_win_over_free_rays_across_packets_in_same_window) {
    auto config=settings(); config.free_space_minimum_observations=2;
    vista::application::RoomMapAccumulator map(config);
    const auto person=ray_frame({map_point(2.2F,0.25F,0.25F)});
    const auto wall=ray_frame({map_point(6.2F,0.25F,0.25F)});
    map.integrate(person,0ms); map.integrate(person,100ms);
    for(int window=2;window<10;++window) {
        map.integrate(wall,std::chrono::milliseconds(window*100));
        map.integrate(person,std::chrono::milliseconds(window*100+50));
    }
    map.integrate({1,{}},1000ms);
    VISTA_CHECK(map.status().point_count==2);
    VISTA_CHECK(map.status().cleared_voxels==0);
}

VISTA_TEST(room_map_uses_world_sensor_origin_and_handles_negative_ray_direction) {
    auto config=settings(); config.free_space_minimum_observations=2;
    vista::application::RoomMapAccumulator map(config);
    const std::array<float,3> origin{-10.25F,2.25F,0.25F};
    const auto person=ray_frame({map_point(-12.2F,2.25F,0.25F)},origin);
    const auto wall=ray_frame({map_point(-16.2F,2.25F,0.25F)},origin);
    map.integrate(person,0ms); map.integrate(person,100ms);
    map.integrate(wall,200ms); map.integrate(wall,300ms); map.integrate(wall,400ms);
    VISTA_CHECK(map.status().cleared_voxels==1);
    VISTA_CHECK(map.status().point_count==1);
}

VISTA_TEST(room_map_does_not_clear_without_origin_or_beyond_raycast_range) {
    auto config=settings(); config.free_space_minimum_observations=2; config.raycast_maximum_range_m=4.0F;
    vista::application::RoomMapAccumulator map(config);
    const auto far=ray_frame({map_point(20.2F,0.25F,0.25F)});
    const auto middle=ray_frame({map_point(8.2F,0.25F,0.25F)});
    map.integrate(middle,0ms); map.integrate(middle,100ms);
    for(int window=2;window<6;++window) map.integrate(far,std::chrono::milliseconds(window*100));
    VISTA_CHECK(map.status().cleared_voxels==0);
    auto unknown_origin=far; unknown_origin.sensor_origin_world_m.reset();
    map.integrate(unknown_origin,600ms);
    VISTA_CHECK(map.status().missing_origin_messages==1);
    VISTA_CHECK(map.status().point_count==2);
}

VISTA_TEST(room_map_raycast_work_is_bounded_per_time_window) {
    auto config=settings(); config.raycast_maximum_rays_per_window=1;
    vista::application::RoomMapAccumulator map(config);
    const auto cloud=ray_frame({map_point(6.2F,0.25F,0.25F),map_point(8.2F,0.25F,0.25F)});
    map.integrate(cloud,0ms); map.integrate(cloud,50ms);
    VISTA_CHECK(map.status().raycasts==1);
    VISTA_CHECK(map.status().ray_budget_skipped_points==3);
    map.integrate(cloud,100ms);
    VISTA_CHECK(map.status().raycasts==2);
}

VISTA_TEST(room_map_loaded_reference_preserves_points_and_cannot_resume) {
    vista::application::RoomMapAccumulator map(settings());
    const auto cloud=ray_frame({map_point(1.1F,0,0),map_point(1.2F,0,0)}); // Same voxel.
    map.restore(map_geometry(cloud)); map.resume();
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::loaded);
    VISTA_CHECK(map.status().point_count==2);
    VISTA_CHECK(!map.integrate(ray_frame({map_point(6,0,0)}),500ms));
    VISTA_CHECK(map.snapshot().points==map_geometry(cloud).points);
}

VISTA_TEST(room_map_freeze_stops_free_space_clearing_as_well_as_new_hits) {
    auto config=settings(); config.free_space_minimum_observations=2;
    vista::application::RoomMapAccumulator map(config);
    const auto person=ray_frame({map_point(2.2F,0.25F,0.25F)});
    const auto wall=ray_frame({map_point(6.2F,0.25F,0.25F)});
    map.integrate(person,0ms); map.integrate(person,100ms);
    map.integrate(wall,200ms); // Pending free evidence is discarded at freeze.
    map.freeze();
    const auto original=map.snapshot().points;
    for(int window=3;window<10;++window)
        VISTA_CHECK(!map.integrate(wall,std::chrono::milliseconds(window*100)));
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::frozen);
    VISTA_CHECK(map.status().cleared_voxels==0);
    VISTA_CHECK(map.snapshot().points==original);
}

VISTA_TEST(room_map_disabled_worker_serves_empty_status_without_consuming_geometry) {
    vista::platform::MessageBus bus; vista::platform::StopToken stop;
    auto config=settings(); config.enabled=false; config.publish_interval=10ms;
    auto observer=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-disabled-map");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,vista::platform::ThreadConfig("test-disabled",4),stop,config,
        [&](auto,auto failure){error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,ray_frame({map_point(2,0,0)})));
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    const auto received=observer.receive_for(status,500ms);
    stop.request_stop(); bus.close(); worker.join();
    VISTA_CHECK(error.empty());
    VISTA_CHECK(received==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(status->state==vista::models::RoomMapState::disabled);
    VISTA_CHECK(status->point_count==0 && status->received_messages==0);
}

VISTA_TEST(room_map_worker_load_failure_never_rebuilds_or_creates_file) {
    vista::platform::MessageBus bus; vista::platform::StopToken stop;
    auto config=settings(); config.load_existing=true; config.publish_interval=10ms;
    config.file=vista::fs::temp_directory_path()/
        ("vista-missing-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    auto observer=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-load-error");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,vista::platform::ThreadConfig("test-load-failure",4),stop,config,
        [&](auto,auto failure){error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,ray_frame({map_point(2,0,0)})));
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    const auto received=observer.receive_for(status,500ms);
    const auto capture_continues=!stop.is_stop_requested();
    stop.request_stop(); bus.close(); worker.join();
    VISTA_CHECK(error.empty() && capture_continues);
    VISTA_CHECK(received==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(status->state==vista::models::RoomMapState::load_error);
    VISTA_CHECK(status->received_messages==0 && status->point_count==0);
    VISTA_CHECK(!vista::fs::exists(config.file));
}

VISTA_TEST(room_map_worker_loading_never_overwrites_source_snapshot) {
    vista::platform::MessageBus bus; vista::platform::StopToken stop;
    auto config=settings(); config.load_existing=true; config.publish_interval=10ms;
    config.file=vista::fs::temp_directory_path()/
        ("vista-reference-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    const auto original=ray_frame({map_point(1.1F,0,0),map_point(1.2F,0,0)});
    vista::transport::save_room_map_snapshot(config.file,map_geometry(original));
    const auto original_stamp=vista::fs::last_write_time(config.file);
    auto observer=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-loaded-status");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,vista::platform::ThreadConfig("test-readonly-map",4),stop,config,
        [&](auto,auto failure){error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,ray_frame({map_point(2,0,0)})));
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    const auto received=observer.receive_for(status,500ms);
    stop.request_stop(); bus.close(); worker.join();
    VISTA_CHECK(error.empty());
    VISTA_CHECK(received==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(status->state==vista::models::RoomMapState::loaded && status->received_messages==0);
    VISTA_CHECK(status->point_count==2);
    VISTA_CHECK(vista::fs::last_write_time(config.file)==original_stamp);
    VISTA_CHECK(vista::transport::load_room_map_snapshot(config.file,10).points==map_geometry(original).points);
    vista::fs::remove(config.file);
}

VISTA_TEST(room_map_worker_building_always_saves_on_clean_shutdown) {
    vista::platform::MessageBus bus; vista::platform::StopToken stop;
    auto config=settings(); config.minimum_observations=1; config.publish_interval=10ms;
    config.file=vista::fs::temp_directory_path()/
        ("vista-auto-save-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    auto observer=bus.subscribe<vista::models::RoomMapMessage>(vista::models::topics::room_map,"test-auto-save-map");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,vista::platform::ThreadConfig("test-map-auto-save",4),stop,config,
        [&](auto,auto failure){error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,ray_frame({map_point(2,0,0)})));
    std::shared_ptr<const vista::models::RoomMapMessage> snapshot;
    bool confirmed=false;
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while(std::chrono::steady_clock::now()<deadline) {
        if(observer.receive_for(snapshot,100ms)==vista::platform::ReceiveStatus::message && !snapshot->payload.points.empty()) {
            confirmed=true; break;
        }
    }
    stop.request_stop(); bus.close(); worker.join();
    VISTA_CHECK(error.empty() && confirmed);
    VISTA_CHECK(vista::fs::exists(config.file));
    VISTA_CHECK(vista::transport::load_room_map_snapshot(config.file,10).points==snapshot->payload.points);
    vista::fs::remove(config.file);
}

VISTA_TEST(room_map_worker_no_data_does_not_overwrite_an_older_map) {
    vista::platform::MessageBus bus; vista::platform::StopToken stop;
    auto config=settings(); config.publish_interval=10ms;
    config.file=vista::fs::temp_directory_path()/
        ("vista-preserve-map-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
    const auto original=ray_frame({map_point(1.1F,0,0)});
    vista::transport::save_room_map_snapshot(config.file,map_geometry(original));
    const auto original_stamp=vista::fs::last_write_time(config.file);
    auto observer=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-map-no-data");
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,vista::platform::ThreadConfig("test-map-no-data",4),stop,config,
        [&](auto,auto failure){error=std::move(failure);});
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    const auto received=observer.receive_for(status,500ms);
    stop.request_stop(); bus.close(); worker.join();
    VISTA_CHECK(error.empty() && received==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(status->point_count==0);
    VISTA_CHECK(vista::fs::last_write_time(config.file)==original_stamp);
    VISTA_CHECK(vista::transport::load_room_map_snapshot(config.file,10).points==map_geometry(original).points);
    vista::fs::remove(config.file);
}

VISTA_TEST(room_map_worker_republishes_same_unchanged_snapshot_for_late_subscribers) {
    vista::platform::MessageBus bus;
    vista::platform::StopToken stop;
    auto config=settings(); config.minimum_observations=1; config.publish_interval=10ms;
    config.freeze_after=0ms;
    auto observer=bus.subscribe<vista::models::RoomMapMessage>(vista::models::topics::room_map,"test-map-observer");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::optional<vista::application::RoomMapReport> report;
    std::string error;
    auto worker=vista::application::spawn_room_map_worker(bus,
        vista::platform::ThreadConfig("test-room-map",4),stop,config,
        [&](auto value, auto failure){report=std::move(value);error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,{1,{map_point(1,0,0)}}));
    std::shared_ptr<const vista::models::RoomMapMessage> snapshot;
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while (std::chrono::steady_clock::now()<deadline) {
        if(observer.receive_for(snapshot,100ms)==vista::platform::ReceiveStatus::message && !snapshot->payload.points.empty()) break;
    }
    auto late=bus.subscribe<vista::models::RoomMapMessage>(vista::models::topics::room_map,"test-late-map");
    std::shared_ptr<const vista::models::RoomMapMessage> replay;
    const auto received=late.receive_for(replay,500ms);
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty());
    VISTA_CHECK(report && report->map_points==1);
    VISTA_CHECK(received==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(replay==snapshot);
}

VISTA_TEST(room_map_worker_skips_unused_legacy_previews) {
    vista::platform::MessageBus bus;
    vista::platform::StopToken stop;
    auto config=settings();config.minimum_observations=1;config.publish_interval=10ms;
    auto status_input=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-status");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string error;
    std::optional<vista::application::RoomMapReport> report;
    auto worker=vista::application::spawn_room_map_worker(bus,
        vista::platform::ThreadConfig("test-no-preview",4),stop,config,
        [&](auto value,auto failure){report=std::move(value);error=std::move(failure);});
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,{1,{map_point(1,0,0)}}));
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    bool confirmed=false;
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while(std::chrono::steady_clock::now()<deadline) {
        if(status_input.receive_for(status,100ms)==vista::platform::ReceiveStatus::message && status->point_count==1) {
            confirmed=true;break;
        }
    }
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(confirmed && error.empty());
    VISTA_CHECK(status->preview_queries==0 && status->view_queries==0);
    VISTA_CHECK(report && report->published_snapshots==0 && report->map_points==1);
}

VISTA_TEST(room_map_worker_publishes_disk_ready_views_on_one_checkpoint_cadence) {
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto config=settings();config.minimum_observations=1;config.publish_interval=250ms;
    auto observer=bus.subscribe<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view,"test-checkpoint-view");
    auto status_input=bus.subscribe<vista::models::RoomMapStatus>(vista::models::topics::room_map_status,"test-checkpoint-status");
    auto producer=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned);
    std::string failure;
    auto worker=vista::application::spawn_room_map_worker(bus,{"test-map-checkpoint",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    std::shared_ptr<const vista::models::RoomMapViewMessage> initial,changed,heartbeat;
    VISTA_CHECK(observer.receive_for(initial,1s)==vista::platform::ReceiveStatus::message);
    producer.publish(vista::devices::LidarPointCloudMessage("test",1,std::nullopt,1,
        {1,{map_point(1.25F,0.25F,0.25F),map_point(3.25F,0.25F,0.25F)}}));
    // Integration does not publish a new disk view for every incoming frame.
    VISTA_CHECK(observer.receive_for(changed,50ms)==vista::platform::ReceiveStatus::timeout);
    VISTA_CHECK(observer.receive_for(changed,1s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(changed->revision>initial->revision && changed->point_count==2 && changed->source);
    vista::models::RoomMapViewRequest request;request.point_budget=10;
    VISTA_CHECK(changed->source->select_view(request).points.size()==2); // Disk is ready BEFORE publication.
    std::shared_ptr<const vista::models::RoomMapStatus> status;
    const auto status_deadline=std::chrono::steady_clock::now()+1s;
    while(std::chrono::steady_clock::now()<status_deadline) {
        if(status_input.receive_for(status,100ms)==vista::platform::ReceiveStatus::message && status->point_count==2) break;
    }
    VISTA_CHECK(status && status->tile_writes>0);
    const auto writes=status->tile_writes;
    VISTA_CHECK(observer.receive_for(heartbeat,1s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(heartbeat->revision==changed->revision && heartbeat->source==changed->source);
    VISTA_CHECK(status_input.receive_for(status,1s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(status->tile_writes==writes); // No checkpoint writes when geometry is unchanged.
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}
