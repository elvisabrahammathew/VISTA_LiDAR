#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include "config.hpp"
#include "4_Applications/perception/foreground/foreground.hpp"
#include "2_Transport/storage/room_map_storage.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"

namespace {
using namespace vista;
using namespace std::chrono_literals;
struct Region final:models::RoomMapRegion {
    std::vector<models::RoomMapPoint> points;
    mutable std::size_t reads{};
    bool corrupt{};
    void read(const std::function<void(const models::RoomMapPoint&)>& visit) const override {
        ++reads;if(corrupt) throw std::runtime_error("test corrupt region");
        for(const auto& p:points) visit(p);
    }
};
std::uint64_t clock_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
std::shared_ptr<models::RoomMapSurfaceSnapshot> background(std::vector<models::RoomMapPoint> points={}) {
    auto result=std::make_shared<models::RoomMapSurfaceSnapshot>();result->voxel_size=.05F;result->tile_size=1;result->geometry_revision=1;
    std::map<models::RoomMapRegionKey,std::shared_ptr<Region>> regions;
    for(const auto& p:points) {
        const models::RoomMapRegionKey key{static_cast<std::int64_t>(std::floor(p.x)),static_cast<std::int64_t>(std::floor(p.y)),static_cast<std::int64_t>(std::floor(p.z))};
        auto& region=regions[key];if(!region) region=std::make_shared<Region>();region->points.push_back(p);
    }
    for(auto& entry:regions) {entry.second->point_count=entry.second->points.size();entry.second->version=1;result->regions.emplace(entry.first,entry.second);}
    return result;
}
models::ForegroundObservation observation(std::vector<models::ForegroundPoint> points,
    std::shared_ptr<const models::RoomMapSurfaceSnapshot> map,std::uint64_t sequence=1) {
    models::ForegroundObservation in;in.source="test";in.sequence=sequence;in.timestamp_ns=10;
    in.monotonic_ns=1'000'000'000ULL+sequence*100'000'000ULL;in.received_monotonic_ns=clock_ns();in.epoch=1;
    in.points=std::move(points);in.background=std::move(map);return in;
}
application::RoomMapConfig map_config() {
    application::RoomMapConfig c;c.enabled=true;c.surface_snapshots=true;c.voxel_size_m=.1F;c.tile_size_m=1;
    c.minimum_observations=2;c.observation_interval=100ms;c.freeze_after=0ms;c.free_space_minimum_observations=2;return c;
}
application::ForegroundConfig query_only_config() {
    application::ForegroundConfig c;c.outlier_filter_enabled=false;return c;
}
models::PointCloudFrame cloud(float x) {return {1,{{x,.025F,.025F,90,0,0,0}},std::array<float,3>{0,.025F,.025F}};}
struct Finish {
    platform::StopToken& stop;platform::MessageBus& bus;platform::WorkerHandle& worker;
    ~Finish(){stop.request_stop();bus.close();worker.join();}
};
struct SnapshotSource final:models::RoomMapViewSource {
    std::shared_ptr<const models::RoomMapSurfaceSnapshot> snapshot;
    bool fail_snapshot{};
    explicit SnapshotSource(std::shared_ptr<const models::RoomMapSurfaceSnapshot> s):snapshot(std::move(s)){}
    models::RoomMapFrame select_view(const models::RoomMapViewRequest&) const override {throw std::runtime_error("foreground must not query renderer LOD");}
    std::array<double,6> bounds() const override {return {-10,-10,-10,10,10,10};}
    std::shared_ptr<const models::RoomMapSurfaceSnapshot> surface_snapshot() const override {
        if(fail_snapshot)throw std::runtime_error("test snapshot failure");
        return snapshot;
    }
};
models::RoomMapViewMessage checkpoint(std::shared_ptr<const models::RoomMapSurfaceSnapshot> map,
    std::uint64_t stamp=0,std::uint64_t revision=1) {
    models::RoomMapViewMessage value;value.source=std::make_shared<SnapshotSource>(std::move(map));
    value.revision=revision;value.state=models::RoomMapState::building;value.checkpoint_monotonic_ns=stamp?stamp:clock_ns();return value;
}
devices::LidarPointCloudMessage scan(std::uint64_t sequence,float x=3.1F,std::uint64_t receipt=0) {
    const auto now=receipt?receipt:clock_ns();
    return {"test",sequence,std::nullopt,10,cloud(x),now,now,1,now};
}
template<class Predicate> bool wait_frame(platform::TopicSubscriber<models::ForegroundFrame>& input,
    std::shared_ptr<const models::ForegroundFrame>& frame,Predicate predicate) {
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(std::chrono::steady_clock::now()<deadline)
        if(input.receive_for(frame,50ms)==platform::ReceiveStatus::message && predicate(*frame)) return true;
    return false;
}
}

VISTA_TEST(foreground_removes_confirmed_background_and_keeps_new_stationary_points_immediately) {
    application::ForegroundEngine engine({});
    auto map=background({{1.25F,.25F,.25F,4}});
    const auto out=engine.process(observation({{1.25F,.25F,.25F,9},{2.25F,.25F,.25F,80}},map));
    VISTA_CHECK(out.input_points==2 && out.background_points==1 && out.points.size()==1 && out.unknown_points==0);
    VISTA_CHECK(out.points[0].x==2.25F && out.points[0].intensity==80);
}
VISTA_TEST(foreground_tolerance_handles_noise_and_neighbors_across_negative_tile_boundaries) {
    application::ForegroundEngine engine({});auto map=background({{-.02F,.25F,.25F,4}});
    const auto out=engine.process(observation({{.03F,.25F,.25F,9},{.20F,.25F,.25F,90}},map));
    VISTA_CHECK(out.background_points==1 && out.points.size()==1 && out.points[0].x==.20F);
}
VISTA_TEST(foreground_empty_confirmed_map_shows_unmapped_geometry_without_motion_history) {
    application::ForegroundEngine engine({});
    const auto out=engine.process(observation({{1,2,3,4}},background()));
    VISTA_CHECK(out.points.size()==1 && out.background_points==0);
}
VISTA_TEST(foreground_missing_or_failed_background_does_not_show_the_whole_room) {
    application::ForegroundEngine engine({});
    VISTA_CHECK(engine.process(observation({{1,2,3,4}},nullptr)).points.empty());
    auto map=background();map->error="test derived cache failure";
    VISTA_CHECK(engine.process(observation({{1,2,3,4}},map)).points.empty());
}
VISTA_TEST(foreground_only_confirmed_geometry_is_background_and_current_scan_cannot_erase_itself) {
    application::RoomMapAccumulator map(map_config());application::ForegroundEngine engine({});
    const auto empty=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},empty)).points.size()==1);
    map.integrate(cloud(1.1F),0ms);const auto tentative=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},tentative,2)).points.size()==1);
    map.integrate(cloud(1.1F),100ms);const auto confirmed=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},confirmed,3)).points.empty());
    // A held old checkpoint remains immutable, including after future integration.
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},empty,4)).points.size()==1);
}
VISTA_TEST(foreground_person_enters_map_moves_free_space_clears_old_position_and_reenters_map) {
    auto c=map_config();application::RoomMapAccumulator map(c);application::ForegroundEngine engine({});
    map.integrate(cloud(1.1F),0ms);map.integrate(cloud(1.1F),100ms);
    auto frame=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},frame)).points.empty());
    VISTA_CHECK(engine.process(observation({{1.6F,.025F,.025F,1}},frame,2)).points.size()==1);
    map.integrate(cloud(3.1F),200ms);map.integrate(cloud(3.1F),300ms);
    map.integrate(cloud(3.1F),400ms); // Commits the second COMPLETE free window; partial windows never clear geometry.
    frame=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.1F,.025F,.025F,1}},frame,3)).points.size()==1);
    map.integrate(cloud(1.6F),500ms);map.integrate(cloud(1.6F),600ms);
    frame=map.view_source()->surface_snapshot();
    VISTA_CHECK(engine.process(observation({{1.6F,.025F,.025F,1}},frame,4)).points.empty());
    VISTA_CHECK(map.status().cleared_voxels>=1);
}
VISTA_TEST(foreground_uses_full_evicted_map_regions_not_renderer_lod_or_resident_map_cache) {
    auto c=map_config();c.voxel_size_m=1;c.tile_size_m=1;c.cache_max_voxels=1;c.cache_max_tiles=1;
    c.minimum_observations=1;c.preview_max_points=1;c.lod_points_per_node=1;
    application::RoomMapAccumulator map(c);
    for(std::size_t i=0;i<3;++i) map.integrate({1,{{static_cast<float>(i*2)+.25F,.25F,.25F,9,0,0,0}}},std::chrono::milliseconds(i*100));
    application::ForegroundEngine engine({});const auto before=map.status();
    const auto out=engine.process(observation({{.25F,.25F,.25F,1},{2.25F,.25F,.25F,1},{4.25F,.25F,.25F,1}},map.view_source()->surface_snapshot()));
    VISTA_CHECK(before.cache_evictions>0 && map.snapshot().points.size()==1);
    VISTA_CHECK(out.points.empty() && out.background_points==3 && out.unknown_points==0);
}
VISTA_TEST(foreground_region_cache_reuses_immutable_pages_and_invalidates_replaced_or_removed_geometry) {
    application::ForegroundEngine engine({});auto map=background({{1.25F,.25F,.25F,1}});
    const auto region=std::dynamic_pointer_cast<const Region>(map->regions.begin()->second);
    for(std::uint64_t i=1;i<5;++i) VISTA_CHECK(engine.process(observation({{1.25F,.25F,.25F,1}},map,i)).points.empty());
    VISTA_CHECK(region->reads==1);
    auto updated=background({{1.75F,.25F,.25F,1}});
    VISTA_CHECK(engine.process(observation({{1.25F,.25F,.25F,1}},updated,5)).points.size()==1);
    VISTA_CHECK(engine.process(observation({{1.75F,.25F,.25F,1}},background(),6)).points.size()==1);
}
VISTA_TEST(foreground_budget_misses_are_unknown_not_false_foreground_and_cache_is_bounded) {
    application::ForegroundConfig c;c.maximum_cached_points=2;c.maximum_cached_regions=2;c.maximum_region_reads_per_frame=1;
    application::ForegroundEngine engine(c);auto map=background({{.25F,.25F,.25F,1},{2.25F,.25F,.25F,1},{4.25F,.25F,.25F,1}});
    std::size_t unknown=0;
    for(std::uint64_t i=1;i<8;++i) {
        const auto out=engine.process(observation({{.25F,.25F,.25F,1},{2.25F,.25F,.25F,1},{4.25F,.25F,.25F,1}},map,i));
        VISTA_CHECK(out.points.empty() && out.cache_points<=2 && out.cache_regions<=2 && out.region_reads<=1);
        unknown+=out.unknown_points;
    }
    VISTA_CHECK(unknown>0);
}
VISTA_TEST(foreground_oversized_or_invalid_regions_never_become_false_new_objects) {
    application::ForegroundConfig c;c.maximum_cached_points=1;application::ForegroundEngine engine(c);
    auto map=background({{1.25F,.25F,.25F,1},{1.5F,.25F,.25F,1}});
    const auto out=engine.process(observation({{1.25F,.25F,.25F,1}},map));
    VISTA_CHECK(out.points.empty() && out.unknown_points==1 && out.cache_points==0);
    auto corrupt=std::make_shared<Region>();corrupt->points={{1.25F,.25F,.25F,1}};corrupt->point_count=1;corrupt->corrupt=true;
    auto broken=background();broken->regions[{1,0,0}]=corrupt;
    VISTA_CHECK_THROWS(engine.process(observation({{1.25F,.25F,.25F,1}},broken,2)));
}
VISTA_TEST(foreground_input_and_coordinate_work_is_bounded) {
    application::ForegroundConfig c;c.maximum_input_points=2;application::ForegroundEngine engine(c);
    const auto out=engine.process(observation({{1,2,3,1},{2,2,3,1},{3,2,3,1},{4,2,3,1},{5,2,3,1}},background()));
    VISTA_CHECK(out.input_points<=2 && out.points.size()<=2);
    VISTA_CHECK(engine.process(observation({{std::numeric_limits<float>::infinity(),0,0,1}},background(),2)).points.empty());
    c.background_distance_m=0;VISTA_CHECK_THROWS(application::ForegroundEngine{c});
}
VISTA_TEST(foreground_dense_duplicate_pcd_comparisons_are_bounded_and_not_false_foreground) {
    // This bucket is diagonally outside the radius, but overlaps the 27-cell
    // neighborhood. Exhaustive duplicate comparisons would add no information.
    std::vector<models::RoomMapPoint> points(2000,{.01F,.01F,.01F,1});
    auto map=background(std::move(points));application::ForegroundEngine engine({});
    const auto out=engine.process(observation({{.14F,.14F,.14F,1}},map));
    VISTA_CHECK(out.points.empty() && out.unknown_points==1 && out.distance_checks<=512);
}
VISTA_TEST(foreground_clock_epoch_gap_and_source_changes_clear_cache_without_replaying_points) {
    application::ForegroundEngine engine({});auto map=background({{1.25F,.25F,.25F,1}});
    auto in=observation({{1.25F,.25F,.25F,1}},map);engine.process(in);
    in.epoch=2;in.sequence=2;in.monotonic_ns+=100'000'000ULL;
    VISTA_CHECK(engine.process(in).points.empty());
    in.monotonic_ns=1;VISTA_CHECK(engine.process(in).points.empty());
    in.source="other";in.monotonic_ns+=2'000'000'000ULL;VISTA_CHECK(engine.process(in).points.empty());
}
VISTA_TEST(foreground_config_shares_map_cache_limits_and_no_longer_requires_motion_thresholds) {
    const std::string base="Lidar: quanergym8\nSensorIP: 127.0.0.1\nSensorPort: 5000\nRoomMapEnabled: 1\n";
    const auto off=parse_device_config_text(base);VISTA_CHECK(!off.foreground.enabled && off.foreground.publish_interval==250ms);
    const auto c=parse_device_config_text(base+"ForegroundEnabled: 1\nGrafanaEnabled: true\nLioEnabled: 1\nGaussianMapEnabled: 0\nForegroundMaxPoints: 1000\nForegroundBackgroundDistanceMeters: 0.12\n");
    VISTA_CHECK(c.room_map.surface_snapshots && c.foreground.require_localization && c.foreground_websocket.enabled);
    VISTA_CHECK(c.foreground.maximum_cached_points==c.room_map.cache_max_voxels && c.foreground.maximum_cached_regions==c.room_map.cache_max_tiles);
    VISTA_CHECK(c.foreground_websocket.maximum_points==1000 && c.foreground_websocket.publish_interval==250ms);
    VISTA_CHECK(parse_device_config_text(base+"ForegroundPublishIntervalMilliseconds: 500\n").foreground.publish_interval==500ms);
    VISTA_CHECK_THROWS(parse_device_config_text(base+"ForegroundPublishIntervalMilliseconds: 49\n"));
    VISTA_CHECK_THROWS(parse_device_config_text(base+"ForegroundEnabled: 1\nRoomMapEnabled: 0\n"));
    VISTA_CHECK_THROWS(parse_device_config_text(base+"ForegroundEnabled: 1\nGrafanaEnabled: true\nForegroundWebSocketPort: 8766\n"));
    VISTA_CHECK_THROWS(parse_device_config_text(base+"ForegroundBackgroundDistanceMeters: 0\n"));
    VISTA_CHECK_THROWS(parse_device_config_text(base+"MotionConfirmationFrames: 3\n"));
}
VISTA_TEST(foreground_worker_rejects_lost_pose_and_recovers_with_valid_fresh_input) {
    platform::MessageBus bus(4);platform::StopToken stop;auto c=query_only_config();c.require_localization=true;c.publish_interval=50ms;
    auto input=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto pose=bus.publisher<models::LocalizationStatus>(models::topics::localization_status);
    auto output=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"test-fg");
    std::optional<application::ForegroundReport> report;std::string error;
    auto worker=application::spawn_foreground_worker(bus,{"test-fg-worker",5},stop,c,[&](auto r,auto e){report=r;error=e;});Finish finish{stop,bus,worker};
    models::LocalizationStatus state;state.state=models::LocalizationState::lost;state.reason="test IMU gap";pose.publish(state);
    maps.publish(checkpoint(background()));input.publish(scan(1));
    std::shared_ptr<const models::ForegroundFrame> frame;bool paused=false;
    for(int i=0;i<20 && !paused;++i) if(output.receive_for(frame,100ms)==platform::ReceiveStatus::message) paused=frame->reason=="localization unavailable: test IMU gap";
    VISTA_CHECK(paused && frame->points.empty());
    state.state=models::LocalizationState::tracking;state.pose_valid=true;state.position_variance=.0001;pose.publish(state);
    input.publish(scan(2));bool seen=false;
    for(int i=0;i<20 && !seen;++i) if(output.receive_for(frame,100ms)==platform::ReceiveStatus::message) seen=frame->points.size()==1;
    VISTA_CHECK(seen);
    input.publish(scan(3,3.1F,clock_ns()-2'000'000'000ULL));
    bool cleared=false;
    for(int i=0;i<20 && !cleared;++i) if(output.receive_for(frame,100ms)==platform::ReceiveStatus::message) cleared=frame->reason=="no fresh valid world scan" && frame->points.empty();
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(cleared && error.empty() && report && report->received_messages==3);
}
VISTA_TEST(foreground_worker_background_query_failure_does_not_stop_lio_or_map_pipeline) {
    platform::MessageBus bus;platform::StopToken stop;
    auto input=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto output=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"failed-fg");
    std::optional<application::ForegroundReport> report;
    auto c=query_only_config();c.publish_interval=50ms;
    auto worker=application::spawn_foreground_worker(bus,{"failed-fg-worker",5},stop,c,[&](auto r,auto){report=r;});Finish finish{stop,bus,worker};
    auto broken=background();auto r=std::make_shared<Region>();r->point_count=1;r->corrupt=true;broken->regions[{1,0,0}]=r;
    maps.publish(checkpoint(broken));input.publish(scan(1,1.25F));
    std::shared_ptr<const models::ForegroundFrame> frame;bool failure=false;
    for(int i=0;i<20 && !failure;++i) if(output.receive_for(frame,100ms)==platform::ReceiveStatus::message) failure=frame->reason=="test corrupt region";
    VISTA_CHECK(failure && frame->points.empty() && !stop.is_stop_requested());
    maps.publish(checkpoint(background(),0,2));input.publish(scan(2,1.25F));bool recovered=false;
    for(int i=0;i<20 && !recovered;++i) if(output.receive_for(frame,100ms)==platform::ReceiveStatus::message) recovered=frame->points.size()==1;
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(recovered && report && report->query_failures==1);
}
VISTA_TEST(foreground_snapshot_acquisition_failure_is_optional_and_recovers) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"snapshot-failure");
    std::optional<application::ForegroundReport> report;std::string error;
    auto worker=application::spawn_foreground_worker(bus,{"snapshot-failure",5},stop,c,[&](auto r,auto e){report=r;error=e;});
    Finish finish{stop,bus,worker};
    auto source=std::make_shared<SnapshotSource>(background());source->fail_snapshot=true;
    auto failed=checkpoint(background());failed.source=source;maps.publish(failed);scans.publish(scan(1));
    std::shared_ptr<const models::ForegroundFrame> frame;
    const bool paused=wait_frame(frames,frame,[](const auto& f){return f.reason.find("test snapshot failure")!=std::string::npos;});
    maps.publish(checkpoint(background(),0,2));scans.publish(scan(2));
    const bool recovered=wait_frame(frames,frame,[](const auto& f){return f.points.size()==1;});
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(paused && recovered && report && error.empty());
    VISTA_CHECK(report->query_failures+report->prefetch_failures>0);
}
VISTA_TEST(foreground_room_map_checkpoint_clock_is_completed_and_stable_on_heartbeats) {
    auto c=map_config();c.minimum_observations=1;c.publish_interval=10ms;
    platform::MessageBus bus;platform::StopToken stop;
    auto producer=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto input=bus.subscribe<models::RoomMapViewMessage>(models::topics::room_map_view,"causal-fg");
    std::string error;auto worker=application::spawn_room_map_worker(bus,{"causal-fg-map",4},stop,c,[&](auto,auto e){error=e;});Finish finish{stop,bus,worker};
    std::shared_ptr<const models::RoomMapViewMessage> first;
    VISTA_CHECK(input.receive_for(first,5s)==platform::ReceiveStatus::message);
    VISTA_CHECK(first->source && first->checkpoint_monotonic_ns>0);
    const auto mono=clock_ns();producer.publish(scan(42,1.1F,mono));
    std::shared_ptr<const models::RoomMapViewMessage> next;bool confirmed=false;
    const auto deadline=std::chrono::steady_clock::now()+5s;
    while(std::chrono::steady_clock::now()<deadline) if(input.receive_for(next,50ms)==platform::ReceiveStatus::message && next->point_count==1){confirmed=true;break;}
    VISTA_CHECK(confirmed && next->checkpoint_monotonic_ns>=mono);
    const auto stamp=next->checkpoint_monotonic_ns,revision=next->revision;
    VISTA_CHECK(input.receive_for(next,5s)==platform::ReceiveStatus::message);
    VISTA_CHECK(next->revision==revision && next->checkpoint_monotonic_ns==stamp);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(error.empty());
}
VISTA_TEST(foreground_loaded_pcd_is_background_on_first_frame_and_never_overwritten) {
    auto c=map_config();c.load_existing=true;c.publish_interval=10ms;
    c.file=fs::temp_directory_path()/("vista-fg-reference-"+std::to_string(clock_ns())+".pcd");
    models::RoomMapFrame original;original.points={{1.1F,.025F,.025F,80}};transport::save_room_map_snapshot(c.file,original);
    const auto stamp=fs::last_write_time(c.file);
    platform::MessageBus bus;platform::StopToken stop;
    auto producer=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto views=bus.subscribe<models::RoomMapViewMessage>(models::topics::room_map_view,"loaded-fg-view");
    auto input=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"loaded-fg");
    auto cfg=query_only_config();cfg.publish_interval=50ms;
    auto fg=application::spawn_foreground_worker(bus,{"loaded-fg-filter",5},stop,cfg,[](auto,auto){});Finish finish_fg{stop,bus,fg};
    std::string error;auto worker=application::spawn_room_map_worker(bus,{"loaded-fg-map",4},stop,c,[&](auto,auto e){error=e;});Finish finish{stop,bus,worker};
    std::shared_ptr<const models::RoomMapViewMessage> loaded;VISTA_CHECK(views.receive_for(loaded,5s)==platform::ReceiveStatus::message);
    producer.publish(scan(42,1.1F));std::shared_ptr<const models::ForegroundFrame> in;
    const bool status=wait_frame(input,in,[](const auto& f){return f.background_points==1;});
    stop.request_stop();bus.close();worker.join();fg.join();
    const bool unchanged=fs::last_write_time(c.file)==stamp && transport::load_room_map_snapshot(c.file,10).points==original.points;
    fs::remove(c.file);
    VISTA_CHECK(error.empty() && status && unchanged && in->points.empty());
}

VISTA_TEST(foreground_250ms_worker_coalesces_latest_scans_without_waiting_for_map_updates) {
    platform::MessageBus bus(4);platform::StopToken stop;auto c=query_only_config();
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"cadence-fg");
    std::optional<application::ForegroundReport> report;
    auto worker=application::spawn_foreground_worker(bus,{"cadence-fg-worker",5},stop,c,[&](auto r,auto){report=r;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background())); // No mapper and no further background publications.
    const auto start=std::chrono::steady_clock::now();std::vector<std::uint64_t> sequences;
    std::uint64_t seq=0;std::shared_ptr<const models::ForegroundFrame> frame;
    while(std::chrono::steady_clock::now()-start<1350ms) {
        scans.publish(scan(++seq));
        while(frames.try_receive(frame)==platform::ReceiveStatus::message) if(!frame->points.empty()) sequences.push_back(frame->sequence);
        std::this_thread::sleep_for(10ms);
    }
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(report && report->received_messages>=20 && report->processed_frames>=3 && report->processed_frames<=5);
    VISTA_CHECK(!sequences.empty() && sequences.back()>report->processed_frames && report->query_failures==0);
}

VISTA_TEST(foreground_worker_uses_newest_completed_background_at_each_sample) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"newest-fg");
    auto worker=application::spawn_foreground_worker(bus,{"newest-fg-worker",5},stop,c,[](auto,auto){});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background()));scans.publish(scan(1));std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==1 && f.points.size()==1;}));
    auto updated=background({{3.1F,.025F,.025F,1}});updated->geometry_revision=2;
    maps.publish(checkpoint(updated,0,2));scans.publish(scan(2));
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==2 && f.background_points==1;}));
    VISTA_CHECK(frame->points.empty() && frame->background_revision==2);
}

VISTA_TEST(foreground_worker_never_subtracts_a_future_or_self_containing_checkpoint) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"future-fg");
    std::optional<application::ForegroundReport> report;
    auto worker=application::spawn_foreground_worker(bus,{"future-fg-worker",5},stop,c,[&](auto r,auto){report=r;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background(),clock_ns()+150'000'000ULL));scans.publish(scan(1));
    std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.reason.find("newer than")!=std::string::npos;}));
    std::this_thread::sleep_for(160ms);scans.publish(scan(2));
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==2 && f.points.size()==1;}));
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(report && report->processed_frames==1);
}

VISTA_TEST(foreground_worker_does_not_reprocess_duplicate_scans_and_clears_on_stale_input) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"duplicate-fg");
    std::optional<application::ForegroundReport> report;
    auto worker=application::spawn_foreground_worker(bus,{"duplicate-fg-worker",5},stop,c,[&](auto r,auto){report=r;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background()));const auto original=scan(1);scans.publish(original);
    std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.points.size()==1;}));
    scans.publish(original);
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.reason=="no fresh valid world scan";}));
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(report && report->received_messages==2 && report->processed_frames==1);
}

VISTA_TEST(foreground_worker_missing_or_failed_map_never_reveals_the_entire_scan) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"missing-map-fg");
    auto worker=application::spawn_foreground_worker(bus,{"missing-map-fg-worker",5},stop,c,[](auto,auto){});Finish finish{stop,bus,worker};
    models::RoomMapViewMessage failed;failed.state=models::RoomMapState::load_error;maps.publish(failed);scans.publish(scan(1));
    std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.reason=="confirmed background unavailable";}));
    VISTA_CHECK(frame->points.empty());maps.publish(checkpoint(background()));scans.publish(scan(2));
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==2 && f.points.size()==1;}));
}

VISTA_TEST(foreground_scan_filter_keeps_supported_first_frame_and_removes_only_isolated_points) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points{{1,0,0,10},{1.05F,0,0,20},{1,.05F,0,30},{3,0,0,40}};
    const auto r=filter.filter(points);
    VISTA_CHECK(r.input_points==4 && r.retained_points==3 && r.isolated_points==1 && r.unresolved_points==0);
    VISTA_CHECK(points.size()==3 && points[0].intensity==10 && points[1].intensity==20 && points[2].intensity==30);
    // No recurrence requirement: an entirely different location passes immediately.
    points={{10,0,0,50},{10.05F,0,0,60},{10,.05F,0,70}};
    VISTA_CHECK(filter.filter(points).retained_points==3);
    points={{100,0,0,80}};VISTA_CHECK(filter.filter(points).isolated_points==1 && points.empty());
    points.clear();VISTA_CHECK(filter.filter(points).input_points==0);
}

VISTA_TEST(foreground_scan_filter_uses_original_neighbors_and_exact_radius_across_negative_cells) {
    application::ForegroundConfig c;c.outlier_radius_m=.125F;
    application::ForegroundScanFilter filter(c);
    std::vector<models::ForegroundPoint> points{{-.125F,0,0,1},{0,0,0,2},{.125F,0,0,3}};
    const auto r=filter.filter(points);
    // Simultaneous radius filter, not iterative erosion: middle point uses both
    // endpoints even though each endpoint has only one neighbor and is removed.
    VISTA_CHECK(r.retained_points==1 && r.isolated_points==2 && points[0].x==0);
    points={{-.02F,-.02F,0,1},{.02F,-.02F,0,2},{0,.02F,0,3}};
    VISTA_CHECK(filter.filter(points).retained_points==3);
}

VISTA_TEST(foreground_scan_filter_duplicates_cannot_manufacture_spatial_support) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points{{0,0,0,1},{0,0,0,2},{.05F,0,0,3},{.05F,0,0,4}};
    VISTA_CHECK(filter.filter(points).isolated_points==4 && points.empty());
    points={{0,0,0,1},{0,0,0,2},{.05F,0,0,3},{0,.05F,0,4}};
    VISTA_CHECK(filter.filter(points).retained_points==4);
}

VISTA_TEST(foreground_scan_filter_has_bounded_work_and_safe_invalid_coordinate_handling) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points(3000,{0,0,0,1});
    const auto r=filter.filter(points);
    VISTA_CHECK(points.empty() && r.distance_checks<=1'000'000 && r.unresolved_points>0);
    VISTA_CHECK(r.isolated_points+r.unresolved_points==r.input_points);
    points={{std::numeric_limits<float>::quiet_NaN(),0,0,1},{std::numeric_limits<float>::infinity(),0,0,1},{1e30F,0,0,1}};
    const auto invalid=filter.filter(points);
    VISTA_CHECK(points.empty() && invalid.unresolved_points==3 && invalid.distance_checks==0);
    application::ForegroundConfig c;c.maximum_input_points=2;application::ForegroundScanFilter small(c);
    points={{0,0,0,1},{.01F,0,0,1},{0,.01F,0,1}};VISTA_CHECK_THROWS(small.filter(points));
}

VISTA_TEST(foreground_scan_filter_is_optional_and_configuration_is_validated) {
    application::ForegroundConfig c;c.outlier_filter_enabled=false;application::ForegroundScanFilter filter(c);
    std::vector<models::ForegroundPoint> points{{3,2,1,99}};
    const auto r=filter.filter(points);VISTA_CHECK(r.retained_points==1 && r.isolated_points==0 && r.distance_checks==0 && points[0].intensity==99);
    const std::string base="Lidar: quanergym8\nSensorIP: 127.0.0.1\nSensorPort: 5000\n";
    const auto parsed=parse_device_config_text(base+"ForegroundOutlierFilterEnabled: 0\nForegroundOutlierRadiusMeters: 0.2\nForegroundOutlierMinNeighbors: 3\n");
    VISTA_CHECK(!parsed.foreground.outlier_filter_enabled && parsed.foreground.outlier_minimum_neighbors==3);
    VISTA_CHECK_NEAR(parsed.foreground.outlier_radius_m,.2F,1e-6);
    VISTA_CHECK(parse_device_config_text(base).foreground.outlier_filter_enabled);
    for(const auto& bad:{"ForegroundOutlierRadiusMeters: 0\n","ForegroundOutlierRadiusMeters: nan\n","ForegroundOutlierRadiusMeters: 2\n","ForegroundOutlierMinNeighbors: 0\n","ForegroundOutlierMinNeighbors: 17\n","ForegroundOutlierFilterEnabled: invalid\n"})
        VISTA_CHECK_THROWS(parse_device_config_text(base+bad));
}

VISTA_TEST(foreground_spatial_filter_does_not_change_room_map_confirmation_or_shared_scan) {
    auto c=map_config();c.minimum_observations=3;
    application::RoomMapAccumulator map(c);application::ForegroundScanFilter filter({});application::ForegroundEngine engine({});
    models::PointCloudFrame original{1,{{1.1F,.025F,.025F,10,0,0,0},{1.21F,.025F,.025F,20,0,0,0},{1.155F,.12F,.025F,30,0,0,0},{3.1F,.025F,.025F,40,0,0,0}}};
    const auto expected=original.points;
    std::vector<models::ForegroundPoint> selected;
    for(const auto& p:original.points) selected.push_back({p.x,p.y,p.z,p.intensity});
    VISTA_CHECK(filter.filter(selected).isolated_points==1);
    const auto out=engine.process(observation(selected,map.view_source()->surface_snapshot()));
    VISTA_CHECK(out.points.size()==3); // Appears on its first sampled frame.
    map.integrate(original,0ms);VISTA_CHECK(map.status().point_count==0);
    map.integrate(original,100ms);VISTA_CHECK(map.status().point_count==0);
    map.integrate(original,200ms);VISTA_CHECK(map.status().point_count==4);
    VISTA_CHECK(original.points==expected); // Includes the isolated point for RoomMap.
    VISTA_CHECK(engine.process(observation(selected,map.view_source()->surface_snapshot(),2)).points.empty());
}

VISTA_TEST(foreground_worker_filters_local_sample_and_publishes_supported_points_on_first_frame) {
    platform::MessageBus bus;platform::StopToken stop;application::ForegroundConfig c;c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"spatial-fg");
    std::optional<application::ForegroundReport> report;std::string error;
    auto worker=application::spawn_foreground_worker(bus,{"spatial-fg-worker",5},stop,c,[&](auto r,auto e){report=r;error=e;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background()));auto source=scan(1);
    source.payload.points={{1,0,0,10,0,0,0},{1.05F,0,0,20,0,0,0},{1,.05F,0,30,0,0,0},{3,0,0,40,0,0,0}};
    scans.publish(source);std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==1 && f.points.size()==3;}));
    VISTA_CHECK(frame->input_points==3 && frame->reason.find("isolated_removed=1")!=std::string::npos);
    VISTA_CHECK(source.payload.points.size()==4 && source.payload.points.back().intensity==40);
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty() && report && report->processed_frames==1 && report->isolated_points==1 && report->query_failures==0);
}

VISTA_TEST(foreground_adaptive_radius_preserves_sparse_far_groups_but_not_near_isolated_points) {
    application::ForegroundScanFilter filter({});
    // Same spacing at 1 m and 3 m: the latter gets 22.5 cm support.
    std::vector<models::ForegroundPoint> points{{1,0,0,1},{1,.17F,0,2},{1,0,.17F,3},
        {3,0,0,4},{3,.17F,0,5},{3,0,.17F,6}};
    const auto r=filter.filter(points,std::array<float,3>{0,0,0});
    VISTA_CHECK(r.retained_points==1 && points[0].intensity==4 && r.adaptive_points==5);
    // Never uses another scan for support and never keeps a lone distant point.
    points={{4,0,0,7}};VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).isolated_points==1);
}

VISTA_TEST(foreground_adaptive_radius_uses_sensor_origin_not_world_origin_and_is_rotation_invariant) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points{{3,0,0,1},{3,.17F,0,2},{3,0,.17F,3}};
    auto translated=points;for(auto& p:translated){p.x+=100;p.y-=40;p.z+=10;}
    VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).retained_points==1);
    VISTA_CHECK(filter.filter(translated,std::array<float,3>{100,-40,10}).retained_points==1);
    VISTA_CHECK(translated[0].intensity==points[0].intensity);
    std::vector<models::ForegroundPoint> rotated{{0,3,0,1},{-.17F,3,0,2},{0,3,.17F,3}};
    VISTA_CHECK(filter.filter(rotated,std::array<float,3>{0,0,0}).retained_points==1);
    // Very large world coordinates do not inflate a near-sensor query radius.
    translated={{101,-40,10,1},{101,-39.83F,10,2},{101,-40,10.17F,3}};
    VISTA_CHECK(filter.filter(translated,std::array<float,3>{100,-40,10}).retained_points==0);
}

VISTA_TEST(foreground_adaptive_radius_is_capped_and_exact_across_negative_cells) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points{{-10,-.125F,0,1},{-10,0,0,2},{-10,.125F,0,3}};
    VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).retained_points==3);
    points={{-10,0,0,1},{-10,.31F,0,2},{-10,0,.31F,3}};
    VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).retained_points==0);
    application::ForegroundConfig c;c.outlier_radius_m=.125F;c.outlier_max_radius_m=.25F;c.outlier_end_distance_m=4;
    application::ForegroundScanFilter exact(c);
    points={{-4,-.25F,0,1},{-4,0,0,2},{-4,.25F,0,3}};
    VISTA_CHECK(exact.filter(points,std::array<float,3>{0,0,0}).retained_points==1 && points[0].intensity==2);
}

VISTA_TEST(foreground_adaptive_missing_or_invalid_origin_falls_back_to_base_radius) {
    application::ForegroundScanFilter filter({});
    const std::vector<models::ForegroundPoint> sparse{{10,0,0,1},{10,.19F,0,2},{10,0,.19F,3}};
    auto points=sparse;const auto missing=filter.filter(points);
    VISTA_CHECK(points.empty() && missing.missing_origin_points==3 && missing.adaptive_points==0);
    points=sparse;const auto invalid=filter.filter(points,std::array<float,3>{0,std::numeric_limits<float>::infinity(),0});
    VISTA_CHECK(points.empty() && invalid.missing_origin_points==3);
    application::ForegroundConfig c;c.outlier_max_radius_m=c.outlier_radius_m;
    application::ForegroundScanFilter fixed(c);points=sparse;
    VISTA_CHECK(fixed.filter(points,std::array<float,3>{0,0,0}).adaptive_points==0 && points.empty());
}

VISTA_TEST(foreground_adaptive_configuration_validation_and_fixed_compatibility) {
    const std::string base="Lidar: quanergym8\nSensorIP: 127.0.0.1\nSensorPort: 5000\n";
    const auto parsed=parse_device_config_text(base+"ForegroundOutlierStartDistanceMeters: 3\nForegroundOutlierEndDistanceMeters: 12\nForegroundOutlierMaxRadiusMeters: 0.4\n");
    VISTA_CHECK_NEAR(parsed.foreground.outlier_start_distance_m,3,1e-6);
    VISTA_CHECK_NEAR(parsed.foreground.outlier_end_distance_m,12,1e-6);
    VISTA_CHECK_NEAR(parsed.foreground.outlier_max_radius_m,.4F,1e-6);
    for(const auto& bad:{"ForegroundOutlierStartDistanceMeters: -1\n","ForegroundOutlierStartDistanceMeters: nan\n",
        "ForegroundOutlierStartDistanceMeters: 10\n","ForegroundOutlierEndDistanceMeters: 1\n",
        "ForegroundOutlierEndDistanceMeters: 1001\n","ForegroundOutlierEndDistanceMeters: inf\n","ForegroundOutlierMaxRadiusMeters: 0\n",
        "ForegroundOutlierMaxRadiusMeters: 1.01\n","ForegroundOutlierMaxRadiusMeters: inf\n"})
        VISTA_CHECK_THROWS(parse_device_config_text(base+bad));
    // Existing configs with larger base remain valid and do not silently shrink.
    auto c=parsed.foreground;c.outlier_radius_m=.5F;c.outlier_max_radius_m=.3F;
    application::ForegroundScanFilter filter(c);
    std::vector<models::ForegroundPoint> points{{10,0,0,1},{10,.3F,0,2},{10,0,.3F,3}};
    VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).retained_points==3);
}

VISTA_TEST(foreground_adaptive_radius_keeps_distinct_neighbor_and_work_budget_guards) {
    application::ForegroundScanFilter filter({});
    std::vector<models::ForegroundPoint> points{{10,0,0,1},{10,0,0,2},{10,.2F,0,3},{10,.2F,0,4}};
    VISTA_CHECK(filter.filter(points,std::array<float,3>{0,0,0}).isolated_points==4 && points.empty());
    points.assign(3000,{10,0,0,1});const auto r=filter.filter(points,std::array<float,3>{0,0,0});
    VISTA_CHECK(points.empty() && r.distance_checks<=1'000'000 && r.unresolved_points>0);
}

VISTA_TEST(foreground_adaptive_linear_start_end_and_max_are_configurable) {
    application::ForegroundConfig c;c.outlier_radius_m=.125F;c.outlier_start_distance_m=1;
    c.outlier_end_distance_m=9;c.outlier_max_radius_m=.25F;application::ForegroundScanFilter filter(c);
    for(const float distance:{0.0F,1.0F,3.0F,5.0F,9.0F,20.0F}){
        const auto radius=.125F+.125F*std::clamp((distance-1)/8,0.0F,1.0F);
        for(const auto delta:{-.001F,.001F}){
            const auto spacing=radius+delta;
            std::vector<models::ForegroundPoint> points{{distance,0,0,1},{distance,spacing,0,2},{distance,-spacing,0,3}};
            filter.filter(points,std::array<float,3>{0,0,0});
            VISTA_CHECK(std::any_of(points.begin(),points.end(),[](const auto& p){return p.intensity==1;})==(delta<0));
        }
    }
}

VISTA_TEST(foreground_worker_passes_sensor_origin_to_adaptive_filter_without_mutating_scan) {
    platform::MessageBus bus;platform::StopToken stop;application::ForegroundConfig c;c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"adaptive-fg");
    std::string error;
    auto worker=application::spawn_foreground_worker(bus,{"adaptive-fg-worker",5},stop,c,[&](auto,auto e){error=e;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background()));auto source=scan(1);
    source.payload.sensor_origin_world_m=std::array<float,3>{100,-40,10};
    source.payload.points={{103,-40,10,10,0,0,0},{103,-39.83F,10,20,0,0,0},{103,-40,10.17F,30,0,0,0}};
    scans.publish(source);std::shared_ptr<const models::ForegroundFrame> frame;
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.sequence==1 && f.points.size()==1;}));
    VISTA_CHECK(frame->reason.find("adaptive_radius_points=3")!=std::string::npos);
    VISTA_CHECK(source.payload.points.size()==3 && source.payload.points.back().intensity==30);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(error.empty());
}

VISTA_TEST(foreground_prefetch_warms_only_current_immutable_pages_with_bounded_reads) {
    application::ForegroundConfig c;c.maximum_region_reads_per_frame=1;
    auto map=background({{1.25F,.25F,.25F,7},{3.25F,.25F,.25F,8}});
    auto in=observation({{1.25F,.25F,.25F,1},{3.25F,.25F,.25F,2}},map);
    application::ForegroundEngine cold(c),warm(c);
    VISTA_CHECK(cold.process(in).unknown_points==1);
    auto first=warm.prefetch(in,std::chrono::steady_clock::now()+1s,100);
    auto second=warm.prefetch(in,std::chrono::steady_clock::now()+1s,100);
    VISTA_CHECK(first.region_reads==1 && second.region_reads==1);
    const auto result=warm.process(in);
    VISTA_CHECK(result.background_points==2 && result.unknown_points==0 && result.region_reads==0);
    VISTA_CHECK(warm.prefetch(in,std::chrono::steady_clock::now()+1s).region_reads==0);
}

VISTA_TEST(foreground_prefetch_never_uses_replaced_or_deleted_background_geometry) {
    application::ForegroundEngine engine({});
    auto first=background({{1.25F,.25F,.25F,7}});
    auto old=observation({{1.25F,.25F,.25F,1}},first);
    VISTA_CHECK(engine.prefetch(old,std::chrono::steady_clock::now()+1s).region_reads==1);
    VISTA_CHECK(engine.process(old).background_points==1);
    auto revised=background({{1.8F,.25F,.25F,8}});revised->geometry_revision=2;
    auto next=observation({{1.25F,.25F,.25F,1}},revised,2);
    engine.prefetch(next,std::chrono::steady_clock::now()+1s);
    VISTA_CHECK(engine.process(next).points.size()==1);
    auto empty=background();next=observation({{1.8F,.25F,.25F,1}},empty,3);
    engine.prefetch(next,std::chrono::steady_clock::now()+1s);
    const auto result=engine.process(next);VISTA_CHECK(result.points.size()==1 && result.cache_points==0);
}

VISTA_TEST(foreground_prefetch_deadline_cache_caps_corruption_and_epoch_reset_are_safe) {
    application::ForegroundConfig c;c.maximum_cached_points=1;c.maximum_cached_regions=1;
    application::ForegroundEngine engine(c);auto map=background({{1.25F,.25F,.25F,7},{3.25F,.25F,.25F,8}});
    auto in=observation({{1.25F,.25F,.25F,1},{3.25F,.25F,.25F,2}},map);
    VISTA_CHECK(engine.prefetch(in,std::chrono::steady_clock::now()-1ms).region_reads==0);
    VISTA_CHECK(engine.prefetch(in,std::chrono::steady_clock::now()+1s,0).region_reads==0);
    const auto warm=engine.prefetch(in,std::chrono::steady_clock::now()+1s,2);
    VISTA_CHECK(warm.region_reads<=2 && warm.cache_points<=1 && warm.cache_regions<=1);
    auto simple=background({{1.25F,.25F,.25F,1}});in=observation({{1.25F,.25F,.25F,1}},simple,2);
    engine.process(in);in.epoch=2;
    VISTA_CHECK(engine.prefetch(in,std::chrono::steady_clock::now()+1s).region_reads==1);
    auto broken=background();auto region=std::make_shared<Region>();region->corrupt=true;region->point_count=1;broken->regions[{1,0,0}]=region;
    in=observation({{1.25F,.25F,.25F,1}},broken,3);
    VISTA_CHECK_THROWS(engine.prefetch(in,std::chrono::steady_clock::now()+1s));
    engine.reset();VISTA_CHECK(engine.process(observation({{1,1,1,1}},background(),4)).points.size()==1);
}

VISTA_TEST(foreground_prefetch_reuses_unchanged_pages_across_checkpoint_heartbeats) {
    application::ForegroundEngine engine({});auto map=background({{1.25F,.25F,.25F,7}});
    auto in=observation({{1.25F,.25F,.25F,1}},map);
    engine.prefetch(in,std::chrono::steady_clock::now()+1s);VISTA_CHECK(engine.process(in).region_reads==0);
    auto same=std::make_shared<models::RoomMapSurfaceSnapshot>(*map);same->geometry_revision++;
    in=observation({{1.25F,.25F,.25F,1}},same,2);
    VISTA_CHECK(engine.prefetch(in,std::chrono::steady_clock::now()+1s).region_reads==0);
    VISTA_CHECK(engine.process(in).background_points==1);
    const auto region=std::dynamic_pointer_cast<const Region>(map->regions.begin()->second);VISTA_CHECK(region->reads==1);
}

VISTA_TEST(foreground_heartbeat_grace_does_not_relax_scan_age_or_real_pose_failure) {
    platform::MessageBus bus;platform::StopToken stop;auto c=query_only_config();c.require_localization=true;c.publish_interval=50ms;
    auto scans=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned);
    auto maps=bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    auto poses=bus.publisher<models::LocalizationStatus>(models::topics::localization_status);
    auto frames=bus.subscribe<models::ForegroundFrame>(models::topics::foreground_points,"heartbeat-grace");
    std::string error;auto worker=application::spawn_foreground_worker(bus,{"heartbeat-grace-worker",5},stop,c,[&](auto,auto e){error=e;});Finish finish{stop,bus,worker};
    maps.publish(checkpoint(background()));models::LocalizationStatus pose;pose.state=models::LocalizationState::tracking;pose.pose_valid=true;pose.position_variance=.0001;poses.publish(pose);
    const auto start=std::chrono::steady_clock::now();std::uint64_t seq=0;bool false_timeout=false,later_output=false,expired=false;
    std::shared_ptr<const models::ForegroundFrame> frame;
    while(std::chrono::steady_clock::now()-start<2800ms){
        scans.publish(scan(++seq));
        if(frames.receive_for(frame,25ms)==platform::ReceiveStatus::message){
            const auto age=std::chrono::steady_clock::now()-start;
            if(age<2200ms && frame->reason.find("localization status older")!=std::string::npos)false_timeout=true;
            if(age>1100ms && age<2200ms && !frame->points.empty())later_output=true;
            if(frame->reason=="localization status older than 2.5 seconds")expired=true;
        }
    }
    poses.publish(pose);scans.publish(scan(++seq));VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return !f.points.empty();}));
    pose.state=models::LocalizationState::lost;pose.pose_valid=false;pose.reason="real fault";poses.publish(pose);
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.points.empty()&&f.reason=="localization unavailable: real fault";}));
    // Even a malformed producer setting valid=true may not pass a held pose.
    pose.state=models::LocalizationState::holding;pose.pose_valid=true;pose.reason="HOLDING last accepted pose";poses.publish(pose);scans.publish(scan(++seq));
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.points.empty()&&f.reason=="localization unavailable: HOLDING last accepted pose";}));
    pose.state=models::LocalizationState::tracking;pose.pose_valid=true;poses.publish(pose);scans.publish(scan(++seq,3.1F,clock_ns()-600'000'000ULL));
    VISTA_CHECK(wait_frame(frames,frame,[](const auto& f){return f.reason=="no fresh valid world scan";}));
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(!false_timeout && later_output && expired && error.empty());
}
