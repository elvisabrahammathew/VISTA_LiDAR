#include "unittest/test.hpp"
#include "config.hpp"
#include "4_Applications/mapping/lio/lio_adapter.hpp"
#include <cmath>
#include <thread>
#include "models/topics.hpp"
#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "1_Platform/timing/measurement_clock.hpp"
#include <type_traits>
#include "1_Platform/workers/workers.hpp"
using namespace vista::application;

VISTA_TEST(lio_synchronizer_retains_overlap_history_with_bounded_buffer) {
    LioSynchronizer sync(0.1);
    for(int i=0;i<=1000;++i)VISTA_CHECK(sync.add({i*0.002,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.take(1.8,1.9));
    VISTA_CHECK(sync.take(1.8996,2.0)); // Real Unitree overlap is up to ~0.382 ms.
    VISTA_CHECK(!sync.take(1.0,1.1)); // History is deliberately finite.
    for(int i=1001;i<=7000;++i)VISTA_CHECK(sync.add({i*0.002,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.size()<=4096);
}

VISTA_TEST(live_worker_continues_when_lio_waits_and_latches_lost_without_map_fallback) {
    vista::platform::MessageBus bus(64);vista::platform::StopToken stop;
    auto input=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto live=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned_sensor,"test-live");
    auto world=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_world,"test-world");
    auto cleaned=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_cleaned,"test-map-input");
    auto state=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"test-live-state");
    PreprocessingConfig config;config.input_world_coordinates=true;config.voxel_size_m.reset();
    GroundRemovalConfig mount;mount.mount_x_m=100;mount.floor_z_m=10;config.mounting=mount;config.ground_removal=mount;
    std::string live_error,clean_error,lio_error;
    auto cleaning=spawn_preprocessing_worker(bus,{"test-world-clean",3},stop,config,[&](auto,auto e){clean_error=e;});
    auto viewing=spawn_live_preprocessing_worker(bus,{"test-live-clean",3},stop,config,[&](auto,auto e){live_error=e;});
    LioConfig lio;lio.enabled=true;lio.synchronization_timeout=std::chrono::milliseconds(30);
    auto localization=spawn_lio_worker(bus,{"test-live-lio",3},stop,lio,[&](auto,auto e){lio_error=e;});
    auto send=[&](std::uint64_t seq,std::uint64_t timestamp) {
        vista::models::PointCloudFrame frame(timestamp,{{1,0,0,1,0,0,timestamp},{2,0,-1,1,0,0,timestamp+10'000'000}});
        input.publish(vista::devices::LidarPointCloudMessage("unitree-l2",seq,timestamp,100,frame,10'000'000'000+seq*100'000'000,10'000'000'000+seq*100'000'000));
    };
    send(0,1'000'000'000); // IMU absent: live still receives floor, in sensor XYZ.
    std::shared_ptr<const vista::devices::LidarPointCloudMessage> first,second,third,message;
    const auto a=live.receive_for(first,std::chrono::seconds(2));
    send(1,900'000'000); // Sensor clock rewind => LOST, not a global stop.
    const auto b=live.receive_for(second,std::chrono::seconds(2));
    std::shared_ptr<const vista::models::LocalizationStatus> diagnostic;bool lost=false;
    for(int i=0;i<3;++i)if(state.receive_for(diagnostic,std::chrono::milliseconds(1100))==vista::platform::ReceiveStatus::message && diagnostic->state==vista::models::LocalizationState::lost){lost=true;break;}
    send(2,1'200'000'000);
    const auto c=live.receive_for(third,std::chrono::seconds(2));
    const bool world_published=world.try_receive(message)==vista::platform::ReceiveStatus::message;
    const bool map_published=cleaned.try_receive(message)==vista::platform::ReceiveStatus::message;
    const bool unexpected_stop=stop.is_stop_requested();
    stop.request_stop();bus.close();localization.join();viewing.join();cleaning.join();
    VISTA_CHECK(a==vista::platform::ReceiveStatus::message && b==a && c==a);
    VISTA_CHECK(first->payload.points.size()==2 && first->payload.points[0].x==1 && first->payload.points[1].z==-1);
    VISTA_CHECK(!first->payload.sensor_origin_world_m);
    VISTA_CHECK(first->measurement_timestamp_ns && first->received_monotonic_ns==10'000'000'000);
    VISTA_CHECK(lost && !world_published && !map_published && !unexpected_stop);
    VISTA_CHECK(live_error.empty() && clean_error.empty() && lio_error.empty());
}

VISTA_TEST(live_websocket_dependencies_do_not_require_ground_processing) {
    vista::AppConfig app;app.grafana.enabled=true;app.pointcloud_websocket.enabled=true;
    auto runtime=app.runtime_config();runtime.threads.ground_processing.enabled=false;
    runtime.threads.pcd_logger.enabled=false;app.pcd_path.reset();
    vista::platform::validate_worker_selection(app,runtime);
    VISTA_CHECK(app.pointcloud_websocket.input_topic==vista::models::topics::pointcloud_cleaned_sensor);
    VISTA_CHECK(!app.pointcloud_websocket.include_ground_status);
    runtime.threads.live_preprocessing.enabled=false;
    VISTA_CHECK_THROWS(vista::platform::validate_worker_selection(app,runtime));
}

using namespace vista::application;
VISTA_TEST(lio_host_clock_preserves_sensor_delta_and_latches_reset) {
    vista::platform::MeasurementClock clock;
    const auto first=clock.align(2'000'000'000,10'000'000'000);
    const auto next=clock.align(2'020'000'000,10'020'000'000);
    VISTA_CHECK(first && next);
    VISTA_CHECK(*next-*first==20'000'000);
    VISTA_CHECK(!clock.align(1'000'000'000,11'000'000'000));
    VISTA_CHECK(!clock.align(3'000'000'000,12'000'000'000));
}
VISTA_TEST(lio_external_clocks_do_not_require_matching_device_epochs) {
    vista::platform::MeasurementClock lidar,imu;
    auto a=lidar.align(1'000'000'000,100'000'000'000);
    auto b=imu.align(9'000'000'000,100'000'000'000);
    VISTA_CHECK(a && b && *a==*b);
}
VISTA_TEST(lio_usb_batched_imu_keeps_sample_period_not_decode_period) {
    vista::platform::MeasurementClock clock;
    auto previous=clock.align(1'000'000'000,10'000'000'000);
    for(std::uint64_t i=1;i<=20;++i) {
        auto next=clock.align(1'000'000'000+i*5'000'000,10'000'000'000+i*100'000);
        VISTA_CHECK(next && previous);
        VISTA_CHECK_NEAR(static_cast<double>(*next-*previous),5'000'000,501);
        previous=next;
    }
}
VISTA_TEST(lio_timestamp_less_imu_uses_monotonic_arrival_and_zero_sensor_epoch_is_valid) {
    vista::platform::MeasurementClock clock;
    VISTA_CHECK(*clock.align(std::nullopt,10'000'000'000)==10'000'000'000);
    VISTA_CHECK(*clock.align(0,10'005'000'000)==10'005'000'000);
    VISTA_CHECK(*clock.align(5'000'000,10'010'000'000)==10'010'000'000);
}
VISTA_TEST(lio_monotonic_clock_rejects_bad_host_time_and_signed_scan_offsets) {
    vista::platform::MeasurementClock clock;
    VISTA_CHECK(clock.align(100,1000));
    VISTA_CHECK(!clock.align(101,999));
    VISTA_CHECK(!clock.align(102,1001));
    VISTA_CHECK(*vista::platform::shift_measurement_time(10'000'000'000,1'000'000'000,1'100'000'000)==9'900'000'000);
    VISTA_CHECK(!vista::platform::shift_measurement_time(1,0,10));
    VISTA_CHECK(!vista::platform::shift_measurement_time(std::nullopt,1,1));
}
VISTA_TEST(lio_imu_envelope_preserves_calendar_time_without_using_it_for_measurement) {
    static_assert(std::is_same_v<vista::models::ImuMessage,vista::devices::LidarImuMessage>);
    vista::models::ImuFrame sample;sample.timestamp_ns=9'000'000'000;
    vista::models::ImuMessage a("external-imu",1,sample.timestamp_ns,100,sample,10'000'000'000,10'000'000'000);
    vista::models::ImuMessage b("external-imu",2,sample.timestamp_ns,50,sample,10'005'000'000,10'005'000'000);
    VISTA_CHECK(b.received_timestamp_ns<a.received_timestamp_ns);
    VISTA_CHECK(*b.measurement_timestamp_ns>*a.measurement_timestamp_ns);
}
VISTA_TEST(lio_synchronization_requires_brackets_and_rejects_gaps) {
    LioSynchronizer sync(0.1);
    VISTA_CHECK(sync.add({0,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.add({0.05,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(!sync.take(0,0.1));
    VISTA_CHECK(sync.add({0.10,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.take(0,0.1).has_value());
    VISTA_CHECK(!sync.add({0.09,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.add({0.5,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(!sync.take(0.1,0.5));
}
VISTA_TEST(lio_world_preprocessing_preserves_pose_and_floor) {
    PreprocessingConfig config;config.input_world_coordinates=true;config.voxel_size_m.reset();
    GroundRemovalConfig mount;mount.mount_x_m=100;config.mounting=mount;
    PointCloudPreprocessor processor(config);
    vista::models::PointCloudFrame frame;frame.sensor_origin_world_m=std::array<float,3>{100,0,1};
    frame.points.push_back({101,0,0});auto clean=processor.clean(frame);
    VISTA_CHECK(clean.points.size()==1);VISTA_CHECK_NEAR(clean.points[0].x,101,1e-6);
    VISTA_CHECK_NEAR((*clean.sensor_origin_world_m)[0],100,1e-6);
    frame.sensor_origin_world_m.reset();VISTA_CHECK_THROWS(processor.clean(frame));
}
namespace {
LioScan scan(double begin,bool room=true) {
    LioScan s;s.begin_s=begin;s.end_s=begin+0.10;s.cloud.timestamp_ns=static_cast<std::uint64_t>((begin+1)*1e9);
    for(int i=0;i<=10;++i)s.imu.push_back({begin+i*0.01,{0,0,9.81},{0,0,0}});
    for(int a=-12;a<=12;++a)for(int b=-12;b<=12;++b) {
        const float x=static_cast<float>(a)*0.15F,y=static_cast<float>(b)*0.15F;
        const std::uint64_t t=s.cloud.timestamp_ns+static_cast<std::uint64_t>((a+12)*3'000'000);
        s.cloud.points.push_back({x,y,-1.0F,1,0,0,t});
        if(room){s.cloud.points.push_back({3.0F,x,y,1,0,0,t});s.cloud.points.push_back({x,3.0F,y,1,0,0,t});}
    }
    return s;
}
}
VISTA_TEST(lio_stationary_room_tracks_and_keeps_ground_in_world_output) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;config.initial_position_m={1,2,1};
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<8;++i)out=engine.process(scan(i*0.10));
    if(out.status.state!=vista::models::LocalizationState::tracking)
        throw std::runtime_error("LIO stationary room: "+out.status.reason+", matches="+std::to_string(out.status.match_ratio)+", residual="+std::to_string(out.status.residual_m));
    VISTA_CHECK(out.status.state==vista::models::LocalizationState::tracking);
    VISTA_CHECK(out.status.pose_valid && out.world_cloud.has_value());
    VISTA_CHECK_NEAR((*out.world_cloud->sensor_origin_world_m)[0],1,0.02);
    bool floor=false;for(const auto& p:out.world_cloud->points)if(std::abs(p.z)<0.01)floor=true;
    VISTA_CHECK(floor);
    VISTA_CHECK(out.status.local_map_points<=config.local_max_points);
}

VISTA_TEST(lio_overlapping_unitree_scans_keep_tracking_without_pose_extrapolation) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<18;++i) {
        auto frame=scan(i*0.0996);
        // Include history back to the previous end, including 500 Hz brackets.
        frame.imu.clear();
        for(int j=-5;j<=55;++j)frame.imu.push_back({frame.begin_s+j*0.002,{0,0,9.81},{0,0,0}});
        out=engine.process(frame);
        if(i>=8){VISTA_CHECK(out.status.pose_valid);VISTA_CHECK(out.world_cloud);}
    }
    VISTA_CHECK(out.status.rejected_scans<=1);
    VISTA_CHECK_NEAR(out.status.position_m[0],0,0.03);
}

VISTA_TEST(lio_lost_keeps_the_original_failure_reason) {
    LioConfig config;config.initialization_samples=20;LioEngine engine(config);
    for(int i=0;i<8;++i)engine.process(scan(i*0.1));
    auto lost=engine.process(scan(0));
    auto next=engine.process(scan(1));
    VISTA_CHECK(next.status.state==vista::models::LocalizationState::lost && !next.world_cloud);
    VISTA_CHECK(next.status.reason.find(lost.status.reason)!=std::string::npos);
}

VISTA_TEST(lio_overlapping_moving_scans_deskew_translation_rotation_and_ray_origins) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    auto position=[](double t){const double u=std::max(0.0,t-0.7);return u<0.4?0.10*u*u:0.016+0.08*(u-0.4);};
    auto yaw=[](double t){return 0.15*std::max(0.0,t-0.7);};
    for(int i=0;i<20;++i) {
        auto frame=scan(i*0.0996);frame.imu.clear();
        for(int j=-5;j<=55;++j) {
            const double t=frame.begin_s+j*0.002,angle=yaw(t);
            const double a=t>=0.7 && t<1.1?0.2:0;
            frame.imu.push_back({t,{a*std::cos(angle),-a*std::sin(angle),9.81},{0,0,t>=0.7?0.15:0}});
        }
        for(auto& p:frame.cloud.points) {
            const double t=frame.begin_s+static_cast<double>(p.timestamp_ns-frame.cloud.timestamp_ns)*1e-9;
            const double angle=yaw(t),x=p.x-position(t),y=p.y;
            p.x=static_cast<float>(std::cos(angle)*x+std::sin(angle)*y);
            p.y=static_cast<float>(-std::sin(angle)*x+std::cos(angle)*y);
        }
        out=engine.process(frame);
    }
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    const double end=19*0.0996+0.1;
    VISTA_CHECK_NEAR(out.status.position_m[0],position(end),0.06);
    VISTA_CHECK_NEAR(out.status.orientation_xyzw[2],std::sin(yaw(end)*0.5),0.03);
    bool wall=false;
    for(const auto& p:out.world_cloud->points) {
        if(std::abs(p.x-3.0F)<0.04F)wall=true;
        VISTA_CHECK(p.ray_origin_world_m.has_value());
        for(float v:*p.ray_origin_world_m)VISTA_CHECK(std::isfinite(v));
    }
    VISTA_CHECK(wall);
}
VISTA_TEST(lio_degenerate_plane_does_not_publish_a_valid_pose) {
    LioConfig config;config.initialization_samples=20;LioEngine engine(config);LioOutput out;
    for(int i=0;i<10;++i)out=engine.process(scan(i*0.1,false));
    VISTA_CHECK(!out.status.pose_valid);VISTA_CHECK(!out.world_cloud);
}
VISTA_TEST(lio_missing_imu_never_falls_back_to_fixed_mount) {
    LioEngine engine(LioConfig{});auto frame=scan(0);frame.imu.clear();auto out=engine.process(frame);
    VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);
}
VISTA_TEST(lio_moving_scan_tracks_translation_and_deskews_room) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    auto position=[](double t){const double u=std::max(0.0,t-0.7);return u<0.4 ? 0.25*u*u : 0.04+0.2*(u-0.4);};
    for(int i=0;i<20;++i) {
        auto frame=scan(i*0.1);
        for(auto& sample:frame.imu) sample.acceleration[0]=(sample.time_s>=0.7 && sample.time_s<1.1)?0.5:0;
        for(auto& p:frame.cloud.points){double t=frame.begin_s+static_cast<double>(p.timestamp_ns-frame.cloud.timestamp_ns)*1e-9;p.x-=static_cast<float>(position(t));}
        out=engine.process(frame);
    }
    if(!out.world_cloud)throw std::runtime_error("moving LIO: "+out.status.reason);
    VISTA_CHECK_NEAR(out.status.position_m[0],position(2.0),0.06);
    VISTA_CHECK_NEAR(out.status.position_m[1],0,0.04);
    bool wall=false;for(const auto& p:out.world_cloud->points)if(std::abs(p.x-3.0F)<0.04F)wall=true;
    VISTA_CHECK(wall);
    VISTA_CHECK(out.world_cloud->points[0].ray_origin_world_m.has_value());
    VISTA_CHECK((*out.world_cloud->points[0].ray_origin_world_m)[0]<(*out.world_cloud->sensor_origin_world_m)[0]);
}
VISTA_TEST(lio_room_map_uses_each_ray_origin_not_the_scan_end_origin) {
    RoomMapConfig config;config.enabled=true;config.voxel_size_m=0.25F;
    config.minimum_observations=1;config.free_space_minimum_observations=2;config.freeze_after=std::chrono::milliseconds(0);
    RoomMapAccumulator map(config);
    vista::models::PointCloudFrame initial(1,{{3.1F,0.1F,0.1F},{10.6F,0.1F,0.1F}},std::array<float,3>{0,0.1F,0.1F});
    map.integrate(initial,std::chrono::milliseconds(0));
    vista::models::PointCloudFrame moved(2,{{12.1F,0.1F,0.1F}},std::array<float,3>{0,0.1F,0.1F});
    moved.points[0].ray_origin_world_m=std::array<float,3>{10.0F,0.1F,0.1F};
    map.integrate(moved,std::chrono::milliseconds(100));map.integrate(moved,std::chrono::milliseconds(200));
    // Free evidence is committed only after its complete observation window.
    map.integrate(moved,std::chrono::milliseconds(300));
    bool behind=false,cleared=true;
    for(const auto& p:map.snapshot().points){if(std::abs(p.x-3.1F)<0.01F)behind=true;if(std::abs(p.x-10.6F)<0.01F)cleared=false;}
    VISTA_CHECK(behind);VISTA_CHECK(cleared);
}
VISTA_TEST(lio_rotating_scan_tracks_yaw_and_preserves_room_geometry) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    auto yaw=[](double t){return 0.2*std::max(0.0,t-0.7);};
    for(int i=0;i<15;++i) {
        auto frame=scan(i*0.1);
        for(auto& sample:frame.imu)sample.angular_velocity[2]=sample.time_s>=0.7?0.2:0;
        for(auto& p:frame.cloud.points) {
            const double t=frame.begin_s+static_cast<double>(p.timestamp_ns-frame.cloud.timestamp_ns)*1e-9;
            const double angle=yaw(t),x=p.x,y=p.y;
            p.x=static_cast<float>(std::cos(angle)*x+std::sin(angle)*y);
            p.y=static_cast<float>(-std::sin(angle)*x+std::cos(angle)*y);
        }
        out=engine.process(frame);
    }
    if(!out.world_cloud)throw std::runtime_error("rotating LIO: "+out.status.reason+", matches="+std::to_string(out.status.match_ratio)+", residual="+std::to_string(out.status.residual_m)+", yaw z="+std::to_string(out.status.orientation_xyzw[2])+", variance="+std::to_string(out.status.position_variance));
    VISTA_CHECK_NEAR(out.status.orientation_xyzw[2],std::sin(yaw(1.5)/2),0.03);
    VISTA_CHECK_NEAR(out.status.position_m[0],0,0.06);
    bool wall=false;for(const auto& p:out.world_cloud->points)if(std::abs(p.x-3.0F)<0.04F)wall=true;
    VISTA_CHECK(wall);
}
VISTA_TEST(lio_stream_reset_latches_lost_without_resetting_world_map) {
    LioConfig config;config.initialization_samples=20;LioEngine engine(config);LioOutput out;
    for(int i=0;i<8;++i)out=engine.process(scan(i*0.1));
    VISTA_CHECK(out.world_cloud.has_value());
    out=engine.process(scan(0));
    VISTA_CHECK(out.status.state==vista::models::LocalizationState::lost && !out.world_cloud);
    out=engine.process(scan(1.0));
    VISTA_CHECK(out.status.state==vista::models::LocalizationState::lost && !out.world_cloud);
}
VISTA_TEST(lio_grafana_measurement_matches_dashboard_fields_and_escapes_reason) {
    vista::models::LocalizationStatus state;
    state.state=vista::models::LocalizationState::tracking;state.pose_valid=true;
    state.position_m={1,2,3};state.reason="match \"ok\"\n";
    const auto line=format_localization_measurement(state);
    VISTA_CHECK(line.find("localization state_code=2i,pose_valid=1i,position_x_m=1")==0);
    for(const auto* key:{"position_y_m=","position_z_m=","match_ratio=","residual_m=","position_variance=","local_map_points=","rejected_scans=","dropped_messages="})
        VISTA_CHECK(line.find(key)!=std::string::npos);
    VISTA_CHECK(line.find("reason=\"match \\\"ok\\\"\"")!=std::string::npos);
    VISTA_CHECK(line.find('\n')==std::string::npos);
}
VISTA_TEST(lio_config_uses_initial_mount_and_separate_extrinsic) {
    auto config=vista::parse_device_config_text("Lidar: quanergy-m8\nSensorIP: 192.168.1.3\nSensorPort: 4141\nLioEnabled: 1\nMountX(lidar): 5\nLioLidarToImuTranslationX: 0.1\n");
    VISTA_CHECK_NEAR(config.lio.initial_position_m[0],5,1e-6);
    VISTA_CHECK_NEAR(config.lio.lidar_to_imu_translation_m[0],0.1,1e-6);
    VISTA_CHECK(config.runtime_config().preprocessing.input_world_coordinates);
    VISTA_CHECK(config.runtime_config().threads.localization.enabled);
    VISTA_CHECK_THROWS(vista::parse_device_config_text("Lidar: quanergy-m8\nSensorIP: 192.168.1.3\nLioMinMatchRatio: 2\n"));
    for(const auto* key:{"LioImuTopic","LioImuSource","LioClockMode","LioImuTimeOffsetMilliseconds"})
        VISTA_CHECK_THROWS(vista::parse_device_config_text(std::string("Lidar: quanergy-m8\nSensorIP: 192.168.1.3\n")+key+": obsolete\n"));
}
VISTA_TEST(lio_worker_external_imu_uses_shared_topic_and_missing_data_gate) {
    vista::platform::MessageBus bus(256);vista::platform::StopToken stop;
    LioConfig config;config.enabled=true;
    config.synchronization_timeout=std::chrono::milliseconds(30);
    auto input=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imu=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto output=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_world,"test-world");
    auto telemetry=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"test-status");
    std::optional<LioReport> report;std::string error;
    auto worker=spawn_lio_worker(bus,vista::platform::ThreadConfig("lio-external-test",3),stop,config,
        [&](auto r,auto e){report=std::move(r);error=std::move(e);});
    auto frame=scan(0);vista::models::ImuFrame sample;sample.timestamp_ns=9'000'000'000;
    sample.linear_acceleration_z_m_s2=9.81F;
    imu.publish(vista::models::ImuMessage("external-imu",0,sample.timestamp_ns,50,sample,10'000'000'000,10'000'000'000));
    const auto sensor_start=frame.cloud.timestamp_ns;
    input.publish(vista::devices::LidarPointCloudMessage("other-lidar",0,sensor_start,100,std::move(frame.cloud),10'000'000'000,10'000'000'000));
    std::shared_ptr<const vista::models::LocalizationStatus> state;
    bool missing_reported=false;
    for(int i=0;i<3;++i)if(telemetry.receive_for(state,std::chrono::milliseconds(1100))==vista::platform::ReceiveStatus::message)
        if(!state->pose_valid && state->reason.find("IMU")!=std::string::npos){missing_reported=true;break;}
    std::shared_ptr<const vista::devices::LidarPointCloudMessage> world;
    const bool published=output.try_receive(world)==vista::platform::ReceiveStatus::message;
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(missing_reported);VISTA_CHECK(!published);VISTA_CHECK(report && error.empty());
}

VISTA_TEST(lio_worker_does_not_mix_two_imu_producers_on_common_topic) {
    vista::platform::MessageBus bus(256);vista::platform::StopToken stop;
    LioConfig config;config.enabled=true;
    auto cloud=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imu=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto telemetry=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"test-mixed-imu");
    auto worker=spawn_lio_worker(bus,vista::platform::ThreadConfig("lio-single-imu-test",3),stop,config,[](auto,auto){});
    vista::models::ImuFrame sample;sample.timestamp_ns=1'000'000'000;sample.linear_acceleration_z_m_s2=9.81F;
    imu.publish(vista::models::ImuMessage("imu-a",0,sample.timestamp_ns,100,sample,10'000'000'000,10'000'000'000));
    sample.timestamp_ns+=5'000'000;
    imu.publish(vista::models::ImuMessage("imu-b",1,sample.timestamp_ns,100,sample,10'005'000'000,10'005'000'000));
    auto frame=scan(0);const auto source=frame.cloud.timestamp_ns;
    cloud.publish(vista::devices::LidarPointCloudMessage("lidar",0,source,100,std::move(frame.cloud),10'000'000'000,10'000'000'000));
    std::shared_ptr<const vista::models::LocalizationStatus> state;bool lost=false;
    for(int i=0;i<3;++i)if(telemetry.receive_for(state,std::chrono::milliseconds(1100))==vista::platform::ReceiveStatus::message)
        if(state->state==vista::models::LocalizationState::lost && !state->pose_valid){lost=true;break;}
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(lost);
}

namespace {
class ClockMockDecoder final : public vista::devices::ILidarDecoder {
public:
    std::optional<vista::models::ImuFrame> decode_imu_packet(const vista::devices::RawPacket& p) override {
        if(p.bytes()[0]!=1)return std::nullopt;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        vista::models::ImuFrame sample;sample.timestamp_ns=*p.timestamp_ns();
        sample.linear_acceleration_z_m_s2=9.81F;return sample;
    }
    vista::models::PointCloudFrame decode_packet(const vista::devices::RawPacket& p) override {
        return vista::models::PointCloudFrame(1'000'000'000,{{1,0,0,0,0,0,1'000'000'000},{1,0,0,0,0,0,*p.timestamp_ns()}});
    }
};
}
VISTA_TEST(lio_decoder_shares_integrated_clock_and_keeps_original_raw_receipt) {
    vista::platform::MessageBus bus(16);vista::platform::StopToken stop;
    auto slot=std::make_shared<vista::devices::LidarDecoderSlot>();slot->replace(std::make_unique<ClockMockDecoder>());
    auto raw=bus.publisher<vista::devices::LidarRawMessage>(vista::models::topics::lidar_raw);
    auto imu=bus.subscribe<vista::models::ImuMessage>(vista::models::topics::lidar_imu,"clock-imu");
    auto cloud=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded,"clock-cloud");
    auto worker=vista::devices::spawn_lidar_decode_worker(bus,vista::platform::ThreadConfig("clock-decode",3),stop,slot,[](auto,auto){});
    raw.publish(vista::devices::LidarRawMessage("unitree-l2",0,1'000'000'000,500,vista::devices::RawPacket({1},1'000'000'000),10'000'000'000));
    raw.publish(vista::devices::LidarRawMessage("unitree-l2",1,1'020'000'000,400,vista::devices::RawPacket({2},1'020'000'000),10'001'000'000));
    std::shared_ptr<const vista::models::ImuMessage> i;
    std::shared_ptr<const vista::devices::LidarPointCloudMessage> c;
    const auto si=imu.receive_for(i,std::chrono::seconds(2));
    const auto sc=cloud.receive_for(c,std::chrono::seconds(2));
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(si==vista::platform::ReceiveStatus::message && sc==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(i->received_monotonic_ns==10'000'000'000 && c->received_monotonic_ns==10'001'000'000);
    VISTA_CHECK(i->received_timestamp_ns==500 && c->received_timestamp_ns==400);
    VISTA_CHECK(i->measurement_timestamp_ns && c->measurement_timestamp_ns);
    VISTA_CHECK_NEAR(static_cast<double>(*c->measurement_timestamp_ns-*i->measurement_timestamp_ns),20'000'000,2001);
    const auto begin=vista::platform::shift_measurement_time(c->measurement_timestamp_ns,c->payload.timestamp_ns,*c->sensor_timestamp_ns);
    VISTA_CHECK(begin);
    VISTA_CHECK_NEAR(static_cast<double>(*begin),static_cast<double>(*i->measurement_timestamp_ns),2001);
}
