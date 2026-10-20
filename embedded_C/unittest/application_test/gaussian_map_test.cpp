#include "unittest/test.hpp"
#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"
#include "2_Transport/storage/room_map_tiles.hpp"
#include "config.hpp"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "4_Applications/mapping/room_map/room_map.hpp"

namespace {
using Store=vista::transport::RoomMapTileStore;
std::unique_ptr<Store> store() {
    auto s=std::make_unique<Store>(vista::fs::path{},.1F,.4F,1000,8,2,2);
    s->enable_surface_snapshots(true);return s;
}
void add(Store& s,int x,int y,int z=0) {
    s.insert({x,y,z},{{(x+.25F)*.1F,(y+.25F)*.1F,(z+.25F)*.1F,77},2,1,0});
}
}
VISTA_TEST(gaussian_regions_are_full_immutable_and_ignore_evidence_only_updates) {
    auto s=store();for(int x=0;x<8;++x) for(int y=0;y<4;++y) add(*s,x,y);
    auto first=s->view_source(1)->surface_snapshot();
    std::size_t count{};for(const auto& r:first->regions) r.second->read([&](const auto&){++count;});
    VISTA_CHECK(count==32); // More than both LOD reservoirs combined.
    const auto revision=first->geometry_revision;
    s->find({0,0,0})->observations++;s->mark_changed({0,0,0});
    auto same=s->view_source(2)->surface_snapshot();
    VISTA_CHECK(same->geometry_revision==revision && same->regions==first->regions);
    s->erase({0,0,0});auto latest=s->view_source(3)->surface_snapshot();
    VISTA_CHECK(latest->geometry_revision>revision);
    count=0;for(const auto& r:first->regions) r.second->read([&](const auto&){++count;});
    VISTA_CHECK(count==32); // Old revision survives atomic page replacement/deletion.
}
VISTA_TEST(gaussian_surfaces_fit_planes_across_tiles_and_propagate_deletions) {
    auto s=store();for(int x=-4;x<8;++x) for(int y=-4;y<8;++y) add(*s,x,y);
    auto snapshot=s->view_source(1)->surface_snapshot();
    vista::application::GaussianMapBuilder builder;
    auto first=builder.update(*snapshot);
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    auto points=first->select_splats(request);VISTA_CHECK(points.size()==144);
    for(const auto& p:points) {
        VISTA_CHECK(p.covariance[0]>0 && p.covariance[3]>0 && p.covariance[5]>0);
        if(p.center.x>0 && p.center.x<.6F && p.center.y>0 && p.center.y<.6F)
            VISTA_CHECK(p.covariance[5]<p.covariance[0]*.1F);
    }
    const auto built=builder.rebuilt_tiles();builder.update(*snapshot);
    VISTA_CHECK(builder.rebuilt_tiles()==built);
    request.point_budget=7;VISTA_CHECK(first->select_splats(request).size()==7);
    for(int x=0;x<4;++x) for(int y=0;y<4;++y) s->erase({x,y,0});
    auto revised=s->view_source(2)->surface_snapshot();
    auto second=builder.update(*revised);request.point_budget=1000;
    VISTA_CHECK(second->select_splats(request).size()==128);
    VISTA_CHECK(first->select_splats(request).size()==144); // In-flight viewer remains coherent.
    VISTA_CHECK(builder.rebuilt_tiles()>built);
    vista::application::GaussianMapBuilder rebuilt;
    const auto fresh=rebuilt.update(*revised)->select_splats(request);
    const auto incremental=second->select_splats(request);
    VISTA_CHECK(fresh.size()==incremental.size());
    for(std::size_t i=0;i<fresh.size();++i) {
        VISTA_CHECK(fresh[i].center==incremental[i].center);
        for(unsigned c=0;c<6;++c) VISTA_CHECK(std::abs(fresh[i].covariance[c]-incremental[i].covariance[c])<1e-7F);
    }
    auto empty=*snapshot;empty.regions.clear();
    VISTA_CHECK(builder.update(empty)->select_splats(request).empty());
}
VISTA_TEST(gaussian_cancelled_update_keeps_last_complete_revision_and_packet_layout) {
    auto s=store();for(int x=0;x<4;++x) for(int y=0;y<4;++y) add(*s,x,y);
    vista::application::GaussianMapBuilder builder;
    auto source=s->view_source(1)->surface_snapshot();auto first=builder.update(*source);
    add(*s,5,0);auto updated=s->view_source(2)->surface_snapshot();
    VISTA_CHECK(!builder.update(*updated,[]{return true;}));
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    VISTA_CHECK(first->select_splats(request).size()==16);
    auto points=builder.update(*updated)->select_splats(request);
    auto packet=vista::application::encode_gaussian_map(points,42,123);
    VISTA_CHECK(packet.size()==32+17*44 && std::memcmp(packet.data(),"GSP1",4)==0);
    std::uint32_t count{},stride{};std::memcpy(&count,packet.data()+8,4);std::memcpy(&stride,packet.data()+12,4);
    VISTA_CHECK(count==17 && stride==44);
}
VISTA_TEST(gaussian_configuration_retires_only_live_socket_and_preserves_sensor_health_path) {
    const auto config=vista::parse_device_config_text("Lidar: UnitreeL2\nConnectionMode: serial\nSerialPort: COM6\nBaudRate: 4000000\nGrafanaEnabled: 1\nRoomMapEnabled: 1\nGaussianMapEnabled: 1\n");
    const auto runtime=config.runtime_config();
    VISTA_CHECK(!config.pointcloud_websocket.enabled && !runtime.threads.pointcloud_websocket.enabled);
    VISTA_CHECK(runtime.threads.live_preprocessing.enabled && runtime.threads.grafana_bridge.enabled);
    VISTA_CHECK(runtime.threads.gaussian_mapping.enabled && runtime.threads.gaussian_map_websocket.enabled);
    VISTA_CHECK(config.room_map.surface_snapshots && config.gaussian_map_websocket.port==8767);
}

VISTA_TEST(gaussian_loaded_reference_preserves_floor_and_deduplicates_repeated_points) {
    vista::application::RoomMapConfig config;config.surface_snapshots=true;
    config.voxel_size_m=.1F;config.tile_size_m=.4F;
    vista::application::RoomMapAccumulator map(config);
    vista::models::RoomMapFrame frame;
    for(int x=-4;x<8;++x) for(int y=-4;y<8;++y) {
        frame.points.push_back({(x+.25F)*.1F,(y+.25F)*.1F,0,33});
        frame.points.push_back(frame.points.back());
    }
    map.restore(frame);
    vista::application::GaussianMapBuilder builder;
    auto view=builder.update(*map.view_source()->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    const auto points=view->select_splats(request);
    VISTA_CHECK(points.size()==144);
    for(const auto& p:points) VISTA_CHECK(p.center.z==0);
}

VISTA_TEST(gaussian_rotated_plane_has_thin_normal_covariance_across_region_boundaries) {
    auto s=store();
    for(int x=-6;x<7;++x) for(int y=-6;y<7;++y) {
        const float px=(x+.25F)*.1F,py=(y+.25F)*.1F,pz=.3F*px+.4F*py;
        s->insert({x,y,static_cast<std::int64_t>(std::floor(pz/.1))},{{px,py,pz,55},2,1,0});
    }
    vista::application::GaussianMapBuilder builder;
    auto view=builder.update(*s->view_source(1)->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    const auto points=view->select_splats(request);VISTA_CHECK(points.size()==169);
    const double length=std::sqrt(1.25),n[3]{-.3/length,-.4/length,1/length};
    unsigned validated{};
    for(const auto& p:points) if(std::abs(p.center.x)<.4 && std::abs(p.center.y)<.4) {
        const auto& c=p.covariance;
        const auto normal_variance=c[0]*n[0]*n[0]+2*c[1]*n[0]*n[1]+2*c[2]*n[0]*n[2]+
            c[3]*n[1]*n[1]+2*c[4]*n[1]*n[2]+c[5]*n[2]*n[2];
        VISTA_CHECK(normal_variance>0 && normal_variance<(c[0]+c[3]+c[5])*.03);
        ++validated;
    }
    VISTA_CHECK(validated>20);
}

VISTA_TEST(gaussian_optional_snapshot_failure_does_not_break_room_map_checkpoint) {
    auto s=store();add(*s,0,0);auto first=s->view_source(1);
    vista::fs::path leaf;
    for(const auto& entry:vista::fs::recursive_directory_iterator(s->directory()))
        if(entry.path().filename()=="cells.bin") leaf=entry.path().parent_path();
    VISTA_CHECK(!leaf.empty());
    // Force only the optional immutable link/copy to fail; the map page is writable.
    vista::fs::create_directories(leaf/"surface-2.bin");
    s->find({0,0,0})->point.intensity=99;s->mark_changed({0,0,0});
    auto latest=s->view_source(2);
    VISTA_CHECK(!latest->surface_snapshot()->error.empty());
    vista::models::RoomMapViewRequest request;request.point_budget=10;
    const auto geometry=latest->select_view(request);
    VISTA_CHECK(geometry.points.size()==1 && geometry.points[0].intensity==99);
    vista::application::GaussianMapBuilder builder;
    VISTA_CHECK_THROWS(builder.update(*latest->surface_snapshot()));
}

// Optional real-map validation without storing a capture/test fixture in Git.
VISTA_TEST(gaussian_optional_real_pcd_fixture) {
    const auto path=std::getenv("VISTA_GAUSSIAN_TEST_PCD");if(!path) return;
    vista::application::RoomMapConfig config;config.surface_snapshots=true;
    vista::application::RoomMapAccumulator map(config);
    const auto started=std::chrono::steady_clock::now();map.load(path);
    auto snapshot=map.view_source()->surface_snapshot();
    vista::application::GaussianMapBuilder builder;
    auto view=builder.update(*snapshot);
    vista::models::RoomMapViewRequest request;request.point_budget=100000;
    const auto points=view->select_splats(request);
    VISTA_CHECK(!points.empty() && points.size()<=request.point_budget);
    for(const auto& p:points) VISTA_CHECK(std::isfinite(p.covariance[0]) && p.covariance[0]>0);
    std::cout<<"Gaussian real PCD: "<<map.status().point_count<<" source points, "<<points.size()
        <<" visible splats, "<<builder.rebuilt_tiles()<<" tiles, "
        <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<" s (load + fit + view)\n";
}
