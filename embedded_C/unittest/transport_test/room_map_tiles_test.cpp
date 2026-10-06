#include <algorithm>
#include <atomic>
#include <chrono>
#include "1_Platform/compat/filesystem.hpp"
#include <fstream>
#include <sstream>
#include <thread>
#include "2_Transport/storage/room_map_tiles.hpp"
#include "4_Applications/mapping/room_map/room_map.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "unittest/test.hpp"

namespace {
using namespace std::chrono_literals;
vista::application::RoomMapConfig config() {
    vista::application::RoomMapConfig result;
    result.voxel_size_m=1;result.tile_size_m=1;result.cache_max_voxels=2;
    result.cache_max_tiles=2;result.minimum_observations=1;result.freeze_after=0ms;
    result.lod_points_per_node=2;result.preview_max_points=4;return result;
}
vista::models::PointXYZIRT point(float x) {return {x,0.25F,0.25F,1,0,0,0};}
vista::models::RoomMapPoint map_geometry(const vista::models::PointXYZIRT& p) {
    return {p.x,p.y,p.z,p.intensity};
}
vista::models::RoomMapFrame map_geometry(const vista::models::PointCloudFrame& cloud) {
    vista::models::RoomMapFrame geometry;geometry.timestamp_ns=cloud.timestamp_ns;
    for(const auto& p:cloud.points) geometry.points.push_back(map_geometry(p));
    return geometry;
}
vista::fs::path test_path() {
    return vista::fs::temp_directory_path()/("vista-tiled-map-test-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".pcd");
}
vista::models::RoomMapViewRequest positive_view(std::size_t budget=20) {
    vista::models::RoomMapViewRequest request;request.point_budget=budget;request.has_camera=true;
    request.camera={0,0,10};request.viewport_height=600;
    request.planes={{{1,0,0,0},{-1,0,0,100},{0,1,0,100},{0,-1,0,100},{0,0,1,100},{0,0,-1,100}}};
    return request;
}
}

VISTA_TEST(room_map_tiles_keep_more_points_than_cache_and_export_complete_pcd) {
    vista::application::RoomMapAccumulator map(config());
    for(int i=0;i<12;++i) map.integrate({1,{point(i+0.25F)}},std::chrono::milliseconds(i*100));
    const auto status=map.status();
    VISTA_CHECK(status.point_count==12 && status.cache_voxels<=2 && status.cache_tiles<=2);
    VISTA_CHECK(status.cache_evictions>0);
    VISTA_CHECK(map.snapshot().points.size()<=4);
    const auto path=test_path();
    {vista::transport::RoomMapSessionFile output(path);map.save(output);}
    const auto full=vista::transport::load_room_map_snapshot(path,100);
    VISTA_CHECK(full.points.size()==12);
    for(int i=0;i<12;++i) VISTA_CHECK(std::find(full.points.begin(),full.points.end(),map_geometry(point(i+0.25F)))!=full.points.end());
    vista::fs::remove(path);
}

VISTA_TEST(room_map_tiles_exceed_old_250000_limit_with_bounded_cache_and_complete_export) {
    auto options=config();options.cache_max_voxels=2000;options.cache_max_tiles=2;
    options.tile_size_m=10;options.lod_points_per_node=4;options.preview_max_points=32;
    vista::application::RoomMapAccumulator map(options);
    // 300 sparse disk pages, 300000 confirmed points, only 2000 resident records.
    // Each input frame stays small; there is no complete input/output map vector.
    for(int tx=0;tx<3;++tx) for(int ty=0;ty<10;++ty) for(int tz=0;tz<10;++tz) {
        vista::models::PointCloudFrame frame;
        for(int x=0;x<10;++x) for(int y=0;y<10;++y) for(int z=0;z<10;++z)
            frame.points.push_back({tx*10+x+0.25F,ty*10+y+0.25F,tz*10+z+0.25F,1,0,0,0});
        map.integrate(frame,0ms);
        VISTA_CHECK(map.status().cache_voxels<=2000 && map.status().cache_tiles<=2);
    }
    VISTA_CHECK(map.status().point_count==300000);
    VISTA_CHECK(map.snapshot().points.size()<=32);
    const auto path=test_path();
    {vista::transport::RoomMapSessionFile output(path);map.save(output);}
    std::uint64_t count{};
    vista::transport::stream_room_map_snapshot(path,[&](const auto&){++count;});
    VISTA_CHECK(count==300000);
    VISTA_CHECK(map.status().cache_voxels<=2000);
    vista::fs::remove(path);
}

VISTA_TEST(room_map_tiles_restore_evicted_hit_evidence_before_confirmation) {
    auto options=config();options.minimum_observations=2;
    vista::application::RoomMapAccumulator map(options);
    for(int i=0;i<6;++i) map.integrate({1,{point(i+0.25F)}},0ms);
    VISTA_CHECK(map.status().point_count==0);
    for(int i=0;i<6;++i) map.integrate({1,{point(i+0.25F)}},100ms);
    VISTA_CHECK(map.status().point_count==6);
    VISTA_CHECK(map.status().cache_voxels<=2);
}

VISTA_TEST(room_map_tiles_camera_culls_and_respects_lod_budget) {
    vista::application::RoomMapAccumulator map(config());
    for(int i=-8;i<8;++i) map.integrate({1,{point(i+0.25F)}},0ms);
    auto request=positive_view();const auto right=map.view_source()->select_view(request);
    VISTA_CHECK(right.points.size()==8);
    for(const auto& p:right.points) VISTA_CHECK(p.x>=0);
    request.point_budget=3;
    VISTA_CHECK(map.view_source()->select_view(request).points.size()<=3);
    request.planes[0]={1,0,0,-1000};
    VISTA_CHECK(map.view_source()->select_view(request).points.empty());
    VISTA_CHECK(map.status().point_count==16); // Camera changes cannot delete storage.
}

VISTA_TEST(room_map_tiles_stream_readonly_pcd_larger_than_cache_and_preserve_duplicates) {
    const auto path=test_path();
    vista::models::PointCloudFrame original;
    for(int i=0;i<10;++i) original.points.push_back(point(i/2+0.25F)); // Deliberate duplicates.
    vista::transport::save_room_map_snapshot(path,map_geometry(original));
    const auto stamp=vista::fs::last_write_time(path);
    vista::application::RoomMapAccumulator map(config());map.load(path);
    VISTA_CHECK(map.status().point_count==10 && map.status().cache_voxels<=2);
    VISTA_CHECK(map.status().state==vista::models::RoomMapState::loaded);
    VISTA_CHECK(!map.integrate({1,{point(100)}},100ms));
    auto request=positive_view(20);const auto frame=map.view_source()->select_view(request);
    VISTA_CHECK(frame.points.size()==10);
    for(const auto& p:original.points)
        VISTA_CHECK(std::count(frame.points.begin(),frame.points.end(),map_geometry(p))==2);
    VISTA_CHECK(vista::fs::last_write_time(path)==stamp);
    vista::fs::remove(path);
}

VISTA_TEST(room_map_tiles_free_space_clears_evicted_geometry_not_occluded_wall) {
    auto options=config();options.free_space_minimum_observations=2;
    vista::application::RoomMapAccumulator map(options);
    const std::array<float,3> origin{0.25F,0.25F,0.25F};
    map.integrate({1,{point(2.25F)},origin},0ms);
    for(int i=0;i<5;++i) map.integrate({1,{point(20.25F+i)}},0ms); // Evict person tile.
    for(int w=1;w<=3;++w) map.integrate({1,{point(6.25F)},origin},std::chrono::milliseconds(w*100));
    VISTA_CHECK(map.status().cleared_voxels==1);
    VISTA_CHECK(map.status().point_count==6);
    for(int w=4;w<=7;++w) map.integrate({1,{point(2.25F)},origin},std::chrono::milliseconds(w*100));
    VISTA_CHECK(map.status().point_count==7); // Previously observed wall remains behind person.
}

VISTA_TEST(room_map_tiles_configuration_rejects_pages_that_cannot_fit_ram) {
    auto options=config();options.tile_size_m=2;
    VISTA_CHECK_THROWS(vista::application::validate_room_map_config(options));
    options=config();options.cache_max_tiles=0;
    VISTA_CHECK_THROWS(vista::application::validate_room_map_config(options));
    options=config();options.lod_points_per_node=0;
    VISTA_CHECK_THROWS(vista::application::validate_room_map_config(options));
}

VISTA_TEST(room_map_stream_export_count_failure_preserves_previous_pcd) {
    const auto path=test_path();vista::models::PointCloudFrame original(1,{point(1)});
    vista::transport::save_room_map_snapshot(path,map_geometry(original));
    VISTA_CHECK_THROWS(vista::transport::write_room_map_snapshot(path,2,[&](const auto& visit){visit(map_geometry(point(2)));}));
    VISTA_CHECK(vista::transport::load_room_map_snapshot(path,10).points==map_geometry(original).points);
    vista::fs::remove(path);auto temporary=path;temporary+=".tmp";vista::fs::remove(temporary);
}

VISTA_TEST(room_map_view_parser_clamps_client_budget_and_rejects_malformed_camera) {
    const std::string valid="VIEW 500 600 0 0 10 1 0 0 100 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100";
    VISTA_CHECK(vista::application::parse_room_map_view_request(valid,100).point_budget==100);
    VISTA_CHECK(vista::application::parse_room_map_view_request(valid,100).has_camera);
    VISTA_CHECK_THROWS(vista::application::parse_room_map_view_request(valid+" extra",100));
    VISTA_CHECK_THROWS(vista::application::parse_room_map_view_request("VIEW 1 nan",100));
    VISTA_CHECK_THROWS(vista::application::parse_room_map_view_request(std::string(2049,'x'),100));
    VISTA_CHECK_THROWS(vista::application::parse_room_map_view_request("VIEW -1 600",100));
    VISTA_CHECK_THROWS(vista::application::parse_room_map_view_request("VIEW 10 600 0 0 0 0 0 0 0",100));
}

VISTA_TEST(room_map_read_only_lookups_do_not_write_pages_or_lod) {
    vista::transport::RoomMapTileStore store({},1,1,8,8,1,4);
    const vista::transport::MapVoxelKey key{0,0,0};
    store.insert(key,{map_geometry(point(0.25F)),1,0,0});store.flush();
    const auto before=store.metrics();
    for(int i=0;i<100;++i) VISTA_CHECK(store.find(key)!=nullptr);
    store.flush();
    VISTA_CHECK(store.metrics().tile_writes==before.tile_writes);
    VISTA_CHECK(store.metrics().lod_node_writes==before.lod_node_writes);
    ++store.find(key)->observations;store.mark_changed(key);store.flush();
    VISTA_CHECK(store.metrics().tile_writes==before.tile_writes+1);
    VISTA_CHECK(store.metrics().lod_node_writes==before.lod_node_writes); // Evidence only, unchanged geometry.
}

VISTA_TEST(room_map_groups_interleaved_hits_to_avoid_tile_thrashing) {
    auto options=config();options.cache_max_voxels=4;options.cache_max_tiles=4;
    vista::application::RoomMapAccumulator map(options);
    vista::models::PointCloudFrame frame;
    for(int repeat=0;repeat<10;++repeat) for(int tile=0;tile<20;++tile)
        frame.points.push_back(point(tile+0.25F));
    map.integrate(frame,0ms);
    VISTA_CHECK(map.status().point_count==20);
    VISTA_CHECK(map.status().cache_evictions==16); // Once per tile, not once per interleaved hit.
    VISTA_CHECK(map.status().integrated_frames==1);
}

VISTA_TEST(room_map_disk_views_do_not_flush_new_ram_hits) {
    auto options=config();options.cache_max_voxels=10;options.cache_max_tiles=10;
    vista::application::RoomMapAccumulator map(options);
    map.integrate({1,{point(1.25F)}},0ms);
    const auto disk=map.view_source();const auto before=map.status().tile_writes;
    map.integrate({2,{point(2.25F)}},100ms);
    VISTA_CHECK(disk->select_view(positive_view()).points.size()==1);
    VISTA_CHECK(map.status().tile_writes==before); // Rendering cannot flush the live cache.
    const auto latest=map.view_source();
    VISTA_CHECK(latest->select_view(positive_view()).points.size()==2);
}

VISTA_TEST(room_map_disk_reader_outlives_accumulator) {
    std::shared_ptr<const vista::models::RoomMapViewSource> reader;
    {
        vista::application::RoomMapAccumulator map(config());
        map.integrate({1,{point(1.25F)}},0ms);reader=map.view_source();
    }
    VISTA_CHECK(reader->select_view(positive_view()).points.size()==1);
}

VISTA_TEST(room_map_concurrent_disk_query_and_resident_updates_are_safe) {
    auto options=config();options.cache_max_voxels=10;options.cache_max_tiles=10;
    vista::application::RoomMapAccumulator map(options);
    vista::models::PointCloudFrame frame;
    for(int i=0;i<5;++i) frame.points.push_back(point(i+0.25F));
    map.integrate(frame,0ms);const auto reader=map.view_source();
    std::atomic<bool> good{true};
    std::thread render([&] {
        try {for(int i=0;i<30;++i) if(reader->select_view(positive_view()).points.size()!=5) good=false;}
        catch(...) {good=false;}
    });
    for(int i=1;i<30;++i) map.integrate(frame,std::chrono::milliseconds(i*100));
    render.join();
    VISTA_CHECK(good.load());
    VISTA_CHECK(map.status().tile_writes==5);
    VISTA_CHECK(map.status().view_queries==30);
}

VISTA_TEST(room_map_reads_legacy_tile_and_lod_sensor_metadata_as_reserved_slots) {
    vista::transport::RoomMapTileStore store({},1,1,8,8,1,4);
    const vista::transport::MapVoxelKey key{0,0,0};
    const auto geometry=map_geometry(point(0.25F));
    store.insert(key,{geometry,1,0,0});
    const auto reader=store.view_source(123);
    std::size_t lod_files{},cell_files{};
    // Only fresh files owned by this test. Populate the old ring/return/time
    // slots with non-zero values; XYZ/intensity and evidence stay untouched.
    for(const auto& entry:vista::fs::recursive_directory_iterator(store.directory())) {
        if(!entry.is_regular_file()) continue;
        const auto name=entry.path().filename().string();
        if(name!="lod.bin" && name!="cells.bin") continue;
        const bool lod=name=="lod.bin";
        VISTA_CHECK(vista::fs::file_size(entry.path())==(lod?39U:83U));
        std::fstream bytes(entry.path().string(),std::ios::binary|std::ios::in|std::ios::out);
        VISTA_CHECK(static_cast<bool>(bytes));
        bytes.seekp(lod?29:49);
        const char old_metadata[10]={7,2,1,2,3,4,5,6,7,8};
        bytes.write(old_metadata,10);bytes.flush();VISTA_CHECK(static_cast<bool>(bytes));
        if(lod) ++lod_files;else ++cell_files;
    }
    VISTA_CHECK(lod_files>0 && cell_files==1);
    vista::models::RoomMapViewRequest request;request.point_budget=10;
    const auto frame=reader->select_view(request);
    VISTA_CHECK(frame.timestamp_ns==123 && frame.points.size()==1);
    VISTA_CHECK(frame.points.front()==geometry);
    std::size_t exported{};
    store.for_each_confirmed([&](const auto& p){VISTA_CHECK(p==geometry);++exported;});
    VISTA_CHECK(exported==1);
}
