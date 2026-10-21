#include "unittest/test.hpp"
#include "config.hpp"
#include "4_Applications/mapping/lio/lio_adapter.hpp"
#include "4_Applications/mapping/lio/core/imu_model.hpp"
#include "4_Applications/mapping/lio/core/imu_robustness.hpp"
#include "4_Applications/mapping/lio/imu_sample_history.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <thread>
#include "models/topics.hpp"
#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "1_Platform/timing/measurement_clock.hpp"
#include <type_traits>
#include "1_Platform/workers/workers.hpp"
#include "4_Applications/mapping/lio/measurement_epoch.hpp"
using namespace vista::application;

VISTA_TEST(lio_duplicate_history_requires_recent_timestamp_and_identical_sample_bits) {
    ImuSampleHistory history;vista::models::ImuFrame sample;
    sample.timestamp_ns=1'000'000'000;sample.linear_acceleration_z_m_s2=9.81F;
    VISTA_CHECK(!history.contains_duplicate(sample));history.remember(sample);
    VISTA_CHECK(history.contains_duplicate(sample));
    auto changed=sample;changed.angular_velocity_x_rad_s=.001F;
    VISTA_CHECK(!history.contains_duplicate(changed));
    changed=sample;changed.orientation_z=.001F;
    VISTA_CHECK(!history.contains_duplicate(changed));
    changed=sample;changed.timestamp_ns+=2'000'000;
    VISTA_CHECK(!history.contains_duplicate(changed));history.remember(changed);
    VISTA_CHECK(history.contains_duplicate(sample)); // Retransmission arrives out of order.
    changed.timestamp_ns+=101'000'000;history.remember(changed);
    VISTA_CHECK(!history.contains_duplicate(sample)); // Old epochs are not silently accepted.
    history.clear();sample.timestamp_ns=0;history.remember(sample);
    VISTA_CHECK(!history.contains_duplicate(sample) && history.size()==0);
    for(std::uint64_t i=1;i<=2000;++i){sample.timestamp_ns=i;history.remember(sample);}
    VISTA_CHECK(history.size()==512);
}

namespace {
// Exercises the real worker admission path, including external IMU producers.
void imu_admission_case(int kind) {
    vista::platform::MessageBus bus(128);vista::platform::StopToken stop;
    auto clouds=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imus=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto telemetry=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"imu-admission-status");
    LioConfig config;config.synchronization_timeout=std::chrono::milliseconds(200);
    LioReport report;std::string error;
    auto worker=spawn_lio_worker(bus,{"imu-admission-test",3},stop,config,[&](auto r,auto e){if(r)report=*r;error=e;});
    vista::models::PointCloudFrame cloud(1'000'000'000,{{1,0,0,1,0,0,1'000'000'000},{1,1,0,1,0,0,1'010'000'000}});
    clouds.publish(vista::devices::LidarPointCloudMessage("lidar",0,cloud.timestamp_ns,100,cloud,10'000'000'000,10'000'000'000));
    auto send=[&](std::uint64_t time,float acceleration,const std::string& source="imu") {
        vista::models::ImuFrame frame;frame.timestamp_ns=time;frame.linear_acceleration_z_m_s2=acceleration;
        const auto normalized=9'000'000'000+time;
        imus.publish(vista::models::ImuMessage(source,time,time,100,frame,normalized+100'000'000,normalized));
    };
    for(std::uint64_t i=0;i<10;++i)send(1'000'000'000+i*2'000'000,9.81F);
    if(kind==0)send(1'012'000'000,9.81F); // Confirmed old duplicate, six ms late.
    if(kind==1)send(1'012'000'000,9.82F); // Same time, conflicting data remains fatal.
    if(kind==2)send(1'011'000'000,9.81F); // Unknown reordered sample remains fatal.
    if(kind==3)send(500'000'000,9.81F);   // Genuine rewind remains fatal.
    if(kind==4)send(1'012'000'000,9.81F,"other-imu"); // No cross-producer dedup.
    if(kind==0)send(1'020'000'000,9.81F);
    bool observed=false,lost=false;
    std::shared_ptr<const vista::models::LocalizationStatus> status;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<deadline) {
        if(telemetry.receive_for(status,std::chrono::milliseconds(100))!=vista::platform::ReceiveStatus::message)continue;
        lost=lost || status->state==vista::models::LocalizationState::lost;
        if((kind==0 && status->reason.find("duplicate_imu_samples=1")!=std::string::npos) || (kind!=0 && lost)) {observed=true;break;}
    }
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty() && observed);
    VISTA_CHECK(lost==(kind!=0));
    VISTA_CHECK(report.duplicate_imu_samples==(kind==0?1U:0U));
}
}
VISTA_TEST(lio_worker_discards_confirmed_duplicate_without_lost_or_reinitialization){imu_admission_case(0);}
VISTA_TEST(lio_worker_does_not_hide_conflicting_imu_with_same_timestamp){imu_admission_case(1);}
VISTA_TEST(lio_worker_does_not_hide_unknown_reordering){imu_admission_case(2);}
VISTA_TEST(lio_worker_still_rejects_genuine_imu_epoch_rewind){imu_admission_case(3);}
VISTA_TEST(lio_worker_never_deduplicates_across_imu_producers){imu_admission_case(4);}

VISTA_TEST(point_lio_output_model_matches_upstream_30_dof_noise_and_priors) {
    LioConfig c;
    const auto defaults=lio_detail::imu_process_noise_diagonal(c);
    VISTA_CHECK(defaults.size()==30);
    VISTA_CHECK(std::string(LioEngine::backend_name()).find("Point-LIO")!=std::string::npos);
    for(std::size_t i=0;i<12;++i)VISTA_CHECK(defaults[i]==0);
    for(std::size_t i=0;i<3;++i){VISTA_CHECK(defaults[12+i]==20);VISTA_CHECK(defaults[15+i]==1000);VISTA_CHECK(defaults[18+i]==500);VISTA_CHECK(defaults[21+i]==0);}
    c.imu_gyroscope_covariance=.2;c.imu_acceleration_covariance=.3;
    c.imu_gyroscope_bias_covariance=.004;c.imu_acceleration_bias_covariance=.005;
    const auto q=lio_detail::imu_process_noise_diagonal(c);
    for(std::size_t i=0;i<3;++i) {
        VISTA_CHECK_NEAR(q[i+24],.004,1e-12);VISTA_CHECK_NEAR(q[i+27],.005,1e-12);
        VISTA_CHECK_NEAR(q[i+15],1000,1e-12); // R=.2/.3 must NOT replace output Q.
    }
    const auto p=lio_detail::imu_initial_covariance_diagonal();
    for(std::size_t i=0;i<p.size();++i) {
        const double expected=(i>=6 && i<12)||(i>=21 && i<24)?.0001:i>=24?.001:.01;
        VISTA_CHECK_NEAR(p[i],expected,1e-12);
    }
}
VISTA_TEST(lio_imu_covariances_reject_invalid_values_even_if_disabled) {
    for(auto member:{&LioConfig::imu_acceleration_covariance,&LioConfig::imu_gyroscope_covariance,
                     &LioConfig::imu_acceleration_bias_covariance,&LioConfig::imu_gyroscope_bias_covariance})
        for(double bad:{0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
            LioConfig c;c.*member=bad;VISTA_CHECK_THROWS(validate_lio_config(c));
        }
}

VISTA_TEST(lio_synchronizer_retains_overlap_history_with_bounded_buffer) {
    LioSynchronizer sync(0.1);
    for(int i=0;i<=1000;++i)VISTA_CHECK(sync.add({i*0.002,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.take(1.8,1.9));
    VISTA_CHECK(sync.take(1.8996,2.0)); // Real Unitree overlap is up to ~0.382 ms.
    VISTA_CHECK(sync.take(1.0,1.1)); // Examining/rejecting a batch is non-consuming.
    sync.retain_from(2.0);
    VISTA_CHECK(!sync.take(1.0,1.1)); // History is deliberately finite.
    for(int i=1001;i<=7000;++i)VISTA_CHECK(sync.add({i*0.002,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.size()<=4096);
}
VISTA_TEST(lio_synchronizer_keeps_unintegrated_history_after_rejected_batch) {
    LioSynchronizer sync(0.1);
    for(int i=0;i<=200;++i)VISTA_CHECK(sync.add({i*0.01,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.take(0.2,1.5));
    VISTA_CHECK(sync.take(0.2,1.8));
    sync.retain_from(0.4);
    VISTA_CHECK(sync.take(0.2,1.8));
    sync.retain_from(1.8);
    VISTA_CHECK(!sync.take(0.2,1.8));
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
    // Explicit legacy API use in this unit test; production no longer starts it.
    runtime.threads.pointcloud_websocket.enabled=true;
    runtime.threads.pcd_logger.enabled=false;app.pcd_path.reset();
    vista::platform::validate_worker_selection(app,runtime);
    VISTA_CHECK(app.pointcloud_websocket.input_topic==vista::models::topics::pointcloud_cleaned_sensor);
    VISTA_CHECK(!app.pointcloud_websocket.include_ground_status);
    runtime.threads.live_preprocessing.enabled=false;
    VISTA_CHECK_THROWS(vista::platform::validate_worker_selection(app,runtime));
}

using namespace vista::application;
VISTA_TEST(lio_host_clock_preserves_sensor_delta_and_announces_reset_epoch) {
    vista::platform::MeasurementClock clock;
    const auto first=clock.align(2'000'000'000,10'000'000'000);
    const auto next=clock.align(2'020'000'000,10'020'000'000);
    VISTA_CHECK(first && next);
    VISTA_CHECK(*next-*first==20'000'000);
    VISTA_CHECK(clock.epoch()==0 && clock.epoch_started_ns()==0);
    VISTA_CHECK(*clock.align(1'000'000'000,11'000'000'000)==11'000'000'000);
    VISTA_CHECK(clock.epoch()==1 && clock.epoch_started_ns()==11'000'000'000);
    const auto after=clock.align(1'020'000'000,11'020'000'000);
    VISTA_CHECK(after && *after==11'020'000'000);
    // Another reboot can be recovered without resetting the host clock.
    VISTA_CHECK(clock.align(1,12'000'000'000));
    VISTA_CHECK(clock.epoch()==2 && clock.epoch_started_ns()==12'000'000'000);
}
VISTA_TEST(lio_host_clock_small_interleaving_never_starts_a_new_epoch) {
    vista::platform::MeasurementClock clock;
    VISTA_CHECK(clock.align(2'000'000'000,10'000'000'000));
    VISTA_CHECK(clock.align(1'999'000'000,10'000'000'001));
    VISTA_CHECK(clock.epoch()==0);
}
VISTA_TEST(lio_epoch_gate_coordinates_shared_clock_and_rejects_old_queues) {
    MeasurementEpochGate gate;using R=MeasurementEpochGate::Result;
    VISTA_CHECK(gate.observe(0,0,0,100)==R::accept);
    VISTA_CHECK(gate.observe(0,1,200,200)==R::changed);
    VISTA_CHECK(gate.observe(1,0,0,199)==R::stale);
    VISTA_CHECK(gate.observe(1,1,200,210)==R::accept); // Same reset, not twice.
    VISTA_CHECK(gate.observe(0,0,0,220)==R::stale); // Delayed old epoch.
    VISTA_CHECK(gate.observe(0,2,300,300)==R::changed);
    VISTA_CHECK(gate.observe(1,2,300,310)==R::accept);
}
VISTA_TEST(lio_epoch_gate_supports_independent_external_imu_epochs) {
    MeasurementEpochGate gate;using R=MeasurementEpochGate::Result;
    VISTA_CHECK(gate.observe(0,1,200,200)==R::changed);
    VISTA_CHECK(gate.observe(1,0,0,210)==R::accept); // LiDAR did not reboot.
    VISTA_CHECK(gate.observe(1,1,300,300)==R::changed);
    VISTA_CHECK(gate.observe(0,1,200,310)==R::accept); // IMU did not reboot again.
    VISTA_CHECK(gate.observe(0,2,400,400)==R::changed);
}
VISTA_TEST(lio_epoch_gate_never_accepts_invalid_boundary_metadata) {
    MeasurementEpochGate gate;using R=MeasurementEpochGate::Result;
    VISTA_CHECK(gate.observe(0,0,100,100)==R::invalid);
    VISTA_CHECK(gate.observe(0,1,0,100)==R::invalid);
    VISTA_CHECK(gate.observe(0,1,200,100)==R::invalid);
    VISTA_CHECK(gate.observe(0,1,100,100)==R::changed);
    VISTA_CHECK(gate.observe(0,1,110,120)==R::invalid);
    VISTA_CHECK(gate.observe(0,2,100,130)==R::invalid);
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
    static_assert(!std::is_same_v<vista::models::ImuMessage,vista::models::LidarMessage<vista::models::ImuFrame>>);
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
    // Moving-map input remains independent of ground removal.
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
    // Ideal acceleration remains unchanged by initialization normalization.
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

VISTA_TEST(lio_borrowed_scan_matches_owned_input_without_relabeling_source_cloud) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;
    LioEngine owned(c),borrowed(c);std::size_t accepted{};
    for(int i=0;i<14;++i){
        auto input=scan(i*.1);auto source=input.cloud;
        source.timestamp_ns+=1234; // A transport timestamp need not be point-time origin.
        const auto before=source.points;
        const auto a=owned.process(input);
        const auto b=borrowed.process(LioScanView{source,input.begin_s,input.end_s,input.imu,input.cloud.timestamp_ns});
        VISTA_CHECK(a.status.state==b.status.state && a.status.reason==b.status.reason);
        VISTA_CHECK(a.status.position_m==b.status.position_m && a.status.orientation_xyzw==b.status.orientation_xyzw);
        VISTA_CHECK(a.status.position_variance==b.status.position_variance);
        VISTA_CHECK(a.world_cloud.has_value()==b.world_cloud.has_value());
        VISTA_CHECK(source.points==before && source.timestamp_ns==input.cloud.timestamp_ns+1234);
        if(a.world_cloud){++accepted;VISTA_CHECK(a.world_cloud->timestamp_ns==b.world_cloud->timestamp_ns);VISTA_CHECK(a.world_cloud->points==b.world_cloud->points);VISTA_CHECK(a.world_cloud->sensor_origin_world_m==b.world_cloud->sensor_origin_world_m);
            for(std::size_t j=0;j<a.world_cloud->points.size();++j)VISTA_CHECK(a.world_cloud->points[j].ray_origin_world_m==b.world_cloud->points[j].ray_origin_world_m);}
    }
    VISTA_CHECK(accepted>3);
}

VISTA_TEST(lio_sort_fast_path_matches_stable_order_with_equal_time_points) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine ordered(c),scrambled(c);
    for(int i=0;i<14;++i){
        auto a=scan(i*.1),b=a;
        // Reverse time groups, not equal-time points. Stable sorting must retain
        // the exact original order within each acquisition-time group.
        std::reverse(b.cloud.points.begin(),b.cloud.points.end());
        for(auto first=b.cloud.points.begin();first!=b.cloud.points.end();){auto last=first+1;while(last!=b.cloud.points.end() && last->timestamp_ns==first->timestamp_ns)++last;std::reverse(first,last);first=last;}
        const auto x=ordered.process(a),y=scrambled.process(b);
        VISTA_CHECK(x.status.state==y.status.state && x.status.reason==y.status.reason);
        VISTA_CHECK(x.status.position_m==y.status.position_m && x.status.orientation_xyzw==y.status.orientation_xyzw);
        if(x.world_cloud){VISTA_CHECK(y.world_cloud && x.world_cloud->points==y.world_cloud->points);for(std::size_t j=0;j<x.world_cloud->points.size();++j)VISTA_CHECK(x.world_cloud->points[j].ray_origin_world_m==y.world_cloud->points[j].ray_origin_world_m);}
    }
}

VISTA_TEST(point_lio_updates_many_lidar_times_and_each_new_imu_once) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);
    for(int i=0;i<9;++i)engine.process(scan(i*.1));
    const auto before=engine.statistics();const auto out=engine.process(scan(.9));const auto after=engine.statistics();
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK(after.lidar_updates-before.lidar_updates>20); // Not one scan-end EKF correction.
    VISTA_CHECK(after.imu_updates-before.imu_updates==10); // Left bracket is never replayed.
}
VISTA_TEST(point_lio_independent_instances_do_not_share_estimator_or_map_globals) {
    LioConfig a,b;a.initial_position_m={2,0,0};b.initial_position_m={-5,3,0};
    LioEngine first(a),second(b);LioOutput x,y;
    for(int i=0;i<12;++i){x=first.process(scan(i*.1));y=second.process(scan(i*.1));}
    VISTA_CHECK(x.status.pose_valid && y.status.pose_valid);
    VISTA_CHECK_NEAR(x.status.position_m[0],2,.02);VISTA_CHECK_NEAR(y.status.position_m[0],-5,.02);
    VISTA_CHECK_NEAR(y.status.position_m[1],3,.02);
}
VISTA_TEST(point_lio_masks_saturated_axes_without_treating_them_as_motion_inputs) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);
    for(int i=0;i<9;++i)engine.process(scan(i*.1));
    const auto before=engine.statistics();
    auto s=scan(.9);for(auto& m:s.imu){m.angular_velocity[0]=35;m.acceleration[1]=30;}
    const auto out=engine.process(s);const auto after=engine.statistics();
    VISTA_CHECK(after.saturated_imu_axes-before.saturated_imu_axes>=20);
    VISTA_CHECK(out.status.reason.find("non-finite IMU")==std::string::npos);
    LioSynchronizer sync(.1);VISTA_CHECK(sync.add({0,{0,30,9.81},{35,0,0}}));
}
VISTA_TEST(point_lio_accepts_acceleration_in_g_with_explicit_unit_norm) {
    LioConfig c;c.imu_acceleration_norm=1;c.imu_acceleration_saturation=3;c.scan_voxel_m=.1;c.map_voxel_m=.1;
    LioEngine engine(c);LioOutput out;
    for(int i=0;i<12;++i){auto s=scan(i*.1);for(auto& m:s.imu)m.acceleration[2]=1;out=engine.process(s);}
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);VISTA_CHECK_NEAR(out.status.position_m[2],0,.02);
}
VISTA_TEST(point_lio_recovery_holds_map_until_three_consistent_matches) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
    for(int i=0;i<8;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid);
    const auto accepted=out.status;auto broken=scan(.8);for(auto& p:broken.cloud.points)p.x+=30;
    out=engine.process(broken);VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);
    for(int i=9;i<=10;++i){out=engine.process(scan(i*.1));VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);VISTA_CHECK(out.status.local_map_points==accepted.local_map_points);}
    out=engine.process(scan(1.1));VISTA_CHECK(out.status.pose_valid && out.world_cloud);
}

VISTA_TEST(point_lio_epoch_reinitialization_preserves_world_anchor_and_map) {
    LioConfig c;c.initial_position_m={3,2,1};c.scan_voxel_m=.1;c.map_voxel_m=.1;
    LioEngine engine(c);LioOutput out;
    for(int i=0;i<12;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid);const auto accepted=out.status;
    auto reset=engine.restart_initialization();
    VISTA_CHECK(!reset.pose_valid && reset.local_map_points==accepted.local_map_points);
    VISTA_CHECK(!engine.propagation_time_s());
    for(int i=0;i<4;++i){out=engine.process(scan(i*.1));VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);}
    for(int i=4;i<18;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    for(int i=0;i<3;++i)VISTA_CHECK_NEAR(out.status.position_m[i],accepted.position_m[i],.02);
    VISTA_CHECK(out.status.local_map_points>=accepted.local_map_points);
}

VISTA_TEST(point_lio_epoch_reinitialization_does_not_seed_a_different_room) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
    for(int i=0;i<12;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid);const auto map_points=out.status.local_map_points;
    engine.restart_initialization();
    for(int i=0;i<20;++i){
        auto different=scan(i*.1);for(auto& p:different.cloud.points)p.x+=30;
        out=engine.process(different);VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);
        VISTA_CHECK(out.status.local_map_points==map_points);
    }
}

VISTA_TEST(lio_worker_shared_epoch_restarts_once_and_ignores_delayed_old_messages) {
    vista::platform::MessageBus bus(128);vista::platform::StopToken stop;
    auto clouds=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imus=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto status=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"epoch-status");
    auto world=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_world,"epoch-world");
    LioConfig config;std::string error;
    auto worker=spawn_lio_worker(bus,{"epoch-test",3},stop,config,[&](auto,auto e){error=e;});
    vista::models::ImuFrame imu;imu.timestamp_ns=1'000'000'000;imu.linear_acceleration_z_m_s2=9.81F;
    imus.publish(vista::models::ImuMessage("unitree-l2",0,imu.timestamp_ns,100,imu,20'000'000'000,20'000'000'000,1,20'000'000'000));
    auto s=scan(0);
    clouds.publish(vista::devices::LidarPointCloudMessage("unitree-l2",1,s.cloud.timestamp_ns,100,s.cloud,20'100'000'000,20'000'000'000,1,20'000'000'000));
    // Arrive later in the queue, but belong to before the epoch boundary.
    clouds.publish(vista::devices::LidarPointCloudMessage("unitree-l2",2,s.cloud.timestamp_ns,100,s.cloud,19'900'000'000,19'900'000'000));
    imus.publish(vista::models::ImuMessage("unitree-l2",3,imu.timestamp_ns,100,imu,19'900'000'000,19'900'000'000));
    std::shared_ptr<const vista::models::LocalizationStatus> latest;bool seen=false;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(std::chrono::steady_clock::now()<deadline) {
        if(status.receive_for(latest,std::chrono::milliseconds(100))!=vista::platform::ReceiveStatus::message)continue;
        if(latest->reason.find("stale_epoch_messages=2")!=std::string::npos){seen=true;break;}
    }
    std::shared_ptr<const vista::devices::LidarPointCloudMessage> output;
    const bool published=world.try_receive(output)==vista::platform::ReceiveStatus::message;
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty() && seen && !published);
    VISTA_CHECK(latest->reason.find("clock_epoch_recoveries=1")!=std::string::npos);
    VISTA_CHECK(latest->reason.find("reinitializations=1")!=std::string::npos);
    VISTA_CHECK(latest->state!=vista::models::LocalizationState::lost);
}

VISTA_TEST(lio_worker_repeated_clock_resets_resume_world_output_without_origin_jump) {
    vista::platform::MessageBus bus(512);vista::platform::StopToken stop;
    auto clouds=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imus=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto world=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_world,"epoch-resume-world");
    LioConfig config;config.initial_position_m={3,2,1};config.scan_voxel_m=.1;config.map_voxel_m=.1;
    std::string error;auto worker=spawn_lio_worker(bus,{"epoch-resume",3},stop,config,[&](auto,auto e){error=e;});
    bool all_resumed=true,invalid_output=false;std::uint64_t sequence=0;
    for(std::uint64_t epoch=0;epoch<3;++epoch) {
        bool resumed=false;const std::uint64_t base=20'000'000'000+epoch*10'000'000'000;
        const auto boundary=epoch?base+1'000'000'000:0;
        for(int i=0;i<20;++i) {
            auto s=scan(i*.1);const auto source=s.cloud.timestamp_ns;
            for(std::uint64_t j=0;j<=25;++j) {
                vista::models::ImuFrame m;m.timestamp_ns=source+j*4'000'000;m.linear_acceleration_z_m_s2=9.81F;
                imus.publish(vista::models::ImuMessage("unitree-l2",sequence++,m.timestamp_ns,100,m,base+m.timestamp_ns,base+m.timestamp_ns,epoch,boundary));
            }
            const auto cloud_seq=sequence++;
            clouds.publish(vista::devices::LidarPointCloudMessage("unitree-l2",cloud_seq,source,100,std::move(s.cloud),base+source+100'000'000,base+source,epoch,boundary));
            // QEMU/debug scheduling can deliver a previous scan late. Await
            // this sequence, not an arbitrary 100-ms wall-time assumption.
            const auto deadline=std::chrono::steady_clock::now()+
                (i<4?std::chrono::milliseconds(100):
                 i<7?std::chrono::milliseconds(300):std::chrono::milliseconds(3000));
            std::shared_ptr<const vista::devices::LidarPointCloudMessage> output;
            while(std::chrono::steady_clock::now()<deadline) {
                if(world.receive_for(output,std::chrono::milliseconds(50))!=vista::platform::ReceiveStatus::message)continue;
                if(output->sequence<cloud_seq)continue;
                resumed=true;
                if(i<4 || output->measurement_clock_epoch!=epoch || !output->payload.sensor_origin_world_m)invalid_output=true;
                else for(int k=0;k<3;++k)if(std::abs((*output->payload.sensor_origin_world_m)[k]-config.initial_position_m[k])>.03)invalid_output=true;
                break;
            }
        }
        all_resumed=all_resumed && resumed;
    }
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty() && all_resumed && !invalid_output);
}

VISTA_TEST(lio_upstream_imu_model_handles_bias_change_without_stationary_lock) {
    LioConfig config;config.scan_voxel_m=.10;config.map_voxel_m=.10;
    config.stationary_update_enabled=false; // Explicit upstream-only baseline.
    LioEngine engine(config);LioOutput out;double maximum=0;
    for(int i=0;i<100;++i) {
        auto frame=scan(i*.1);
        if(i>=15)for(auto& imu:frame.imu) {
            imu.acceleration[0]=.30;imu.angular_velocity[2]=.05;
        }
        out=engine.process(frame);
        if(i>=25) {
            VISTA_CHECK(out.status.pose_valid && out.world_cloud);
            // Measured gyro change exceeds the rest detector's .03 threshold:
            // this must exercise LiDAR/IMU bias correction, not a fixed pose.
            VISTA_CHECK(out.status.reason.find("stationary velocity constraint")==std::string::npos);
            double squared=0;for(double x:out.status.position_m)squared+=x*x;
            maximum=std::max(maximum,std::sqrt(squared));
        }
    }
    VISTA_CHECK(maximum<.10);
}

VISTA_TEST(point_lio_nonrepeating_surfaces_track_without_fixed_pose_constraint) {
    LioConfig config;config.scan_voxel_m=.05;config.map_voxel_m=.20;
    config.stationary_update_enabled=false; // Preserve the upstream-only regression.
    LioEngine engine(config);std::size_t constrained=0;
    for(int i=0;i<80;++i) {
        auto s=scan(i*.1);
        // Shift sampling along each surface; nearest points no longer repeat.
        const float offset=static_cast<float>((i*7)%13)*.011F;
        for(std::size_t j=0;j<s.cloud.points.size();++j) {
            auto& p=s.cloud.points[j];
            if(j%3==0){p.x+=offset;p.y-=offset;}
            else if(j%3==1){p.y+=offset;p.z-=offset;}
            else {p.x+=offset;p.z-=offset;}
        }
        if(i>=12)for(auto& imu:s.imu)imu.acceleration[0]=.10;
        const auto out=engine.process(s);
        if(i>=12)VISTA_CHECK(out.status.pose_valid);
        if(out.status.reason.find("stationary velocity constraint")!=std::string::npos)++constrained;
        VISTA_CHECK(std::abs(out.status.position_m[0])<.03);
    }
    VISTA_CHECK(constrained==0); // Point-LIO learns motion/bias, no old ZUPT shortcut.
}
VISTA_TEST(lio_rejected_unintegrated_scan_keeps_actual_propagation_horizon) {
    LioEngine engine(LioConfig{});
    for(int i=0;i<8;++i)engine.process(scan(i*.1));
    const auto horizon=engine.propagation_time_s();VISTA_CHECK(horizon);
    auto broken=scan(.8);broken.imu[5].acceleration[0]=std::numeric_limits<double>::infinity();
    VISTA_CHECK(!engine.process(broken).world_cloud);
    VISTA_CHECK(*engine.propagation_time_s()==*horizon);
    auto next=scan(.9);next.imu.clear();
    for(int i=0;i<=20;++i)next.imu.push_back({*horizon+i*.01,{0,0,9.81},{0,0,0}});
    auto out=engine.process(next);
    VISTA_CHECK(!out.status.pose_valid && !out.world_cloud); // First recovery confirmation.
    VISTA_CHECK_NEAR(*engine.propagation_time_s(),1.0,1e-8);
    out=engine.process(scan(1.0));VISTA_CHECK(!out.status.pose_valid && !out.world_cloud);
    out=engine.process(scan(1.1));VISTA_CHECK(out.status.pose_valid && out.world_cloud);
}
VISTA_TEST(lio_constant_velocity_is_not_stationary_even_with_quiet_imu) {
    LioConfig config;config.scan_voxel_m=.10;config.map_voxel_m=.10;
    LioEngine engine(config);LioOutput out;
    const auto position=[](double t){const double u=std::max(0.0,t-1.2);return u<.4?.25*u*u:.04+.2*(u-.4);};
    for(int i=0;i<60;++i) {
        auto s=scan(i*.1);
        for(auto& imu:s.imu)imu.acceleration[0]=(imu.time_s>=1.2 && imu.time_s<1.6)?.5:0;
        for(auto& p:s.cloud.points){const double t=s.begin_s+static_cast<double>(p.timestamp_ns-s.cloud.timestamp_ns)*1e-9;p.x-=static_cast<float>(position(t));}
        out=engine.process(s);
        if(i>=25) {
            VISTA_CHECK(out.status.pose_valid);
            VISTA_CHECK(out.status.reason.find("stationary velocity constraint")==std::string::npos);
        }
    }
    VISTA_CHECK_NEAR(out.status.position_m[0],position(6.0),.06);
}

VISTA_TEST(lio_robustifier_weights_isolated_spikes_without_changing_samples) {
    vista::application::lio_detail::ImuRobustifier gate;
    const std::array<double,6> quiet{0,0,0,0,0,9.81};
    for(int i=0;i<30;++i)gate.observe(quiet);
    auto spike=quiet;spike[0]=.3;spike[4]=4;
    const auto weights=gate.observe(spike);
    VISTA_CHECK(weights[0]>1 && weights[4]>1 && weights[1]==1 && weights[5]==1);
    VISTA_CHECK(spike[0]==.3 && spike[4]==4 && gate.size()==15);
    const auto normal=gate.observe(quiet);for(double w:normal)VISTA_CHECK(w==1);
}
VISTA_TEST(lio_robustifier_never_clips_a_sustained_motion_step) {
    vista::application::lio_detail::ImuRobustifier gate;
    std::array<double,6> sample{0,0,0,0,0,9.81};for(int i=0;i<20;++i)gate.observe(sample);
    sample[2]=.4;sample[3]=2;
    for(int i=0;i<10;++i){const auto w=gate.observe(sample);if(i>=2)VISTA_CHECK(w[2]==1 && w[3]==1);}
}
VISTA_TEST(lio_robustifier_does_not_mistake_alternating_glitches_for_motion_step) {
    vista::application::lio_detail::ImuRobustifier gate;
    const std::array<double,6> quiet{0,0,0,0,0,9.81};
    for(int i=0;i<30;++i)gate.observe(quiet);
    for(int i=0;i<5;++i){auto sample=quiet;sample[0]=i%2?.5:-.5;VISTA_CHECK(gate.observe(sample)[0]>1);}
}
VISTA_TEST(point_lio_initializes_with_stationary_motor_vibration_but_not_changing_means) {
    for(bool moving:{false,true}) {
        LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
        for(int i=0;i<35;++i){auto s=scan(i*.1);s.imu.clear();
            for(int j=0;j<=50;++j){const double t=s.begin_s+j*.002;
                const double wave=std::sin(2*3.141592653589793*60*t);
                s.imu.push_back({t,{moving?.8*std::sin(2*3.141592653589793*2*t):.3*wave,.3*wave,9.81+.6*wave},
                    {.03*std::sin(2*3.141592653589793*70*t),0,0}});
            }
            out=engine.process(s);
        }
        if(moving)VISTA_CHECK(!out.status.pose_valid && !out.world_cloud);
        else VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    }
}
VISTA_TEST(point_lio_joint_rest_evidence_updates_velocity_not_world_pose) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;c.initial_position_m={3,2,1};LioEngine engine(c);LioOutput out;
    for(int i=0;i<70;++i){auto s=scan(i*.1);if(i>12)for(auto&m:s.imu)m.acceleration[0]=.04;out=engine.process(s);}
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK(engine.statistics().stationary_updates>0 && engine.statistics().velocity_norm_m_s<.03);
    for(int i=0;i<3;++i)VISTA_CHECK_NEAR(out.status.position_m[i],c.initial_position_m[i],.05);
}
VISTA_TEST(point_lio_zupt_releases_when_motion_starts_after_confirmed_rest) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
    const auto position=[](double t){const double u=std::max(0.0,t-4);return u<.4?.25*u*u:.04+.2*(u-.4);};
    std::uint64_t rest_updates=0;
    for(int i=0;i<85;++i){auto s=scan(i*.1);
        for(auto&m:s.imu)m.acceleration[0]=(m.time_s>=4 && m.time_s<4.4)?.5:0;
        for(auto&p:s.cloud.points){const auto t=s.begin_s+static_cast<double>(p.timestamp_ns-s.cloud.timestamp_ns)*1e-9;p.x-=static_cast<float>(position(t));}
        out=engine.process(s);if(i==35)rest_updates=engine.statistics().stationary_updates;
        if(i==65)rest_updates=engine.statistics().stationary_updates;
        if(i>65)VISTA_CHECK(engine.statistics().stationary_updates==rest_updates);
    }
    VISTA_CHECK(rest_updates>0 && out.status.pose_valid);VISTA_CHECK_NEAR(out.status.position_m[0],position(8.5),.07);
}
VISTA_TEST(point_lio_zupt_releases_on_rotation_and_does_not_snap_to_mount) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;std::uint64_t count=0;
    for(int i=0;i<65;++i){auto s=scan(i*.1);
        for(auto&m:s.imu)m.angular_velocity[2]=m.time_s>=3?.2:0;
        for(auto&p:s.cloud.points){const auto t=s.begin_s+static_cast<double>(p.timestamp_ns-s.cloud.timestamp_ns)*1e-9;
            const double a=.2*std::max(0.0,t-3);const float x=p.x,y=p.y;p.x=static_cast<float>(std::cos(a)*x+std::sin(a)*y);p.y=static_cast<float>(-std::sin(a)*x+std::cos(a)*y);}
        out=engine.process(s);if(i==45)count=engine.statistics().stationary_updates;
        if(i>45)VISTA_CHECK(engine.statistics().stationary_updates==count);
    }
    VISTA_CHECK(count>0 && out.status.pose_valid);
    VISTA_CHECK_NEAR(2*std::atan2(out.status.orientation_xyzw[2],out.status.orientation_xyzw[3]),.2*(6.5-3),.05);
}
VISTA_TEST(point_lio_rest_layer_does_not_freeze_slow_real_translation) {
    for(double speed:{.01,.03}) {
        LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
        const auto position=[&](double t){const double u=std::max(0.0,t-4);return u<.5?speed*u*u: speed*(u-.25);};
        for(int i=0;i<120;++i){auto s=scan(i*.1);
            for(auto&m:s.imu)m.acceleration[0]=(m.time_s>=4 && m.time_s<4.5)?2*speed:0;
            for(auto&p:s.cloud.points){const auto t=s.begin_s+static_cast<double>(p.timestamp_ns-s.cloud.timestamp_ns)*1e-9;p.x-=static_cast<float>(position(t));}
            out=engine.process(s);
        }
        VISTA_CHECK(out.status.pose_valid && out.world_cloud);
        VISTA_CHECK_NEAR(out.status.position_m[0],position(12),.025);
    }
}
VISTA_TEST(point_lio_epoch_reset_clears_rest_and_noise_evidence_before_requalifying) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
    for(int i=0;i<50;++i)out=engine.process(scan(i*.1));
    const auto updates=engine.statistics().stationary_updates;
    VISTA_CHECK(updates>0 && out.status.pose_valid);
    engine.restart_initialization();
    for(int i=0;i<12;++i){out=engine.process(scan(i*.1));VISTA_CHECK(engine.statistics().stationary_updates==updates);}
    for(int i=12;i<50;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid && engine.statistics().stationary_updates>updates);
}
VISTA_TEST(point_lio_robust_imu_keeps_temporal_coverage_during_spikes) {
    LioConfig c;c.scan_voxel_m=.1;c.map_voxel_m=.1;LioEngine engine(c);LioOutput out;
    for(int i=0;i<60;++i){auto s=scan(i*.1);if(i>=15 && i%4==0){s.imu[5].acceleration[0]=15;s.imu[5].angular_velocity[2]=.8;}out=engine.process(s);}
    VISTA_CHECK(engine.statistics().downweighted_imu_axes>0);
    VISTA_CHECK(out.status.pose_valid && out.world_cloud && engine.statistics().imu_updates>500);
    VISTA_CHECK(std::abs(out.status.position_m[0])<.15);
}
VISTA_TEST(point_lio_noise_and_rest_layers_can_be_disabled_without_extra_estimator) {
    LioConfig c;c.robust_imu_enabled=false;c.stationary_update_enabled=false;LioEngine engine(c);LioOutput out;
    for(int i=0;i<35;++i)out=engine.process(scan(i*.1));
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK(engine.statistics().stationary_updates==0 && engine.statistics().downweighted_imu_axes==0);
}
VISTA_TEST(lio_latest_scan_selection_does_not_hide_stopped_imu_timeout) {
    vista::platform::MessageBus bus(128);vista::platform::StopToken stop;
    auto input=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto imu=bus.publisher<vista::models::ImuMessage>(vista::models::topics::lidar_imu);
    auto status=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"test-stopped-imu-state");
    LioConfig config;config.synchronization_timeout=std::chrono::milliseconds(30);
    std::string error;
    auto worker=spawn_lio_worker(bus,{"test-stopped-imu",3},stop,config,[&](auto,auto e){error=e;});
    for(std::uint64_t i=0;i<=30;++i) {
        vista::models::ImuFrame p;p.timestamp_ns=900'000'000+i*10'000'000;p.linear_acceleration_z_m_s2=9.81F;
        imu.publish(vista::models::ImuMessage("unitree-l2",i,p.timestamp_ns,100,p,9'000'000'000+p.timestamp_ns,9'000'000'000+p.timestamp_ns));
    }
    bool lost=false;
    for(std::uint64_t i=0;i<180 && !lost;++i) {
        auto s=scan(i*.1);
        input.publish(vista::devices::LidarPointCloudMessage("unitree-l2",i,s.cloud.timestamp_ns,100,s.cloud,9'000'000'000+s.cloud.timestamp_ns,9'000'000'000+s.cloud.timestamp_ns));
        std::shared_ptr<const vista::models::LocalizationStatus> message;
        while(status.try_receive(message)==vista::platform::ReceiveStatus::message)
            lost|=message->state==vista::models::LocalizationState::lost && message->reason.find("missing right IMU bracket")!=std::string::npos;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(error.empty() && lost);
}
VISTA_TEST(lio_time_failures_distinguish_reset_missing_history_and_backlog) {
    for(int failure=0;failure<3;++failure) {
        LioEngine engine(LioConfig{});for(int i=0;i<8;++i)engine.process(scan(i*.1));
        const double horizon=*engine.propagation_time_s();
        auto s=scan(failure==0?horizon-.2:failure==1?horizon+.1:horizon+2.1);
        if(failure==2){s.imu.clear();for(int i=0;i<=221;++i)s.imu.push_back({horizon+i*.01,{0,0,9.81},{0,0,0}});}
        const auto out=engine.process(s);
        const char* reason=failure==0?"scan end is not monotonic":failure==1?"IMU batch misses integrated horizon":"propagation backlog exceeds 2s";
        VISTA_CHECK(out.status.state==vista::models::LocalizationState::lost);
        VISTA_CHECK(out.status.reason.find(reason)!=std::string::npos);
        VISTA_CHECK(!out.world_cloud);
    }
}
VISTA_TEST(lio_recovery_timer_requires_five_continuous_host_seconds) {
    LioRecoveryTimer timer;using S=vista::models::LocalizationState;
    const auto epoch=std::chrono::steady_clock::time_point{};
    VISTA_CHECK(!timer.update(S::lost,epoch));
    VISTA_CHECK(!timer.update(S::lost,epoch+std::chrono::milliseconds(4999)));
    VISTA_CHECK(timer.update(S::lost,epoch+std::chrono::seconds(5)));
    VISTA_CHECK(!timer.update(S::initializing,epoch+std::chrono::seconds(6)));
    VISTA_CHECK(!timer.update(S::lost,epoch+std::chrono::seconds(7)));
    VISTA_CHECK(!timer.update(S::degraded,epoch+std::chrono::seconds(11)));
    VISTA_CHECK(!timer.update(S::lost,epoch+std::chrono::seconds(12)));
    VISTA_CHECK(!timer.update(S::lost,epoch+std::chrono::seconds(16)));
    VISTA_CHECK(timer.update(S::lost,epoch+std::chrono::seconds(17)));
}

VISTA_TEST(lio_initialization_window_tolerates_isolated_acceleration_noise) {
    LioConfig config;config.initialization_samples=200;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<45;++i) {
        auto frame=scan(i*0.1);
        for(auto& s:frame.imu) {
            const auto index=static_cast<int>(std::round(s.time_s*100));
            s.acceleration[0]=(index%17==0)?0.65:0.0;
        }
        out=engine.process(frame);
    }
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK_NEAR(out.status.position_m[0],0,0.05);
}

VISTA_TEST(lio_initialization_sample_count_works_at_low_imu_rate) {
    LioConfig config;config.initialization_samples=200;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;bool progress=false;
    for(int i=0;i<65;++i) {
        auto frame=scan(i*0.1);frame.imu.clear();
        for(int j=0;j<=5;++j)frame.imu.push_back({frame.begin_s+j*0.02,{0,0,9.81},{0,0,0}});
        out=engine.process(frame);
        if(i<30){VISTA_CHECK(!out.world_cloud);progress|=out.status.reason.find("samples=")!=std::string::npos;}
    }
    VISTA_CHECK(progress && out.status.pose_valid && out.world_cloud);
}

VISTA_TEST(lio_initialization_reports_motion_and_resumes_after_stable_window) {
    LioConfig config;config.initialization_samples=50;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<12;++i) {
        auto frame=scan(i*0.1);for(auto& s:frame.imu)s.angular_velocity[2]=0.3;
        out=engine.process(frame);VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);
    }
    VISTA_CHECK(out.status.reason.find("gyro_norm=")!=std::string::npos);
    for(int i=12;i<32;++i)out=engine.process(scan(i*0.1));
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
}

VISTA_TEST(point_lio_output_measurements_handle_small_bias_change_without_zupt) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    config.stationary_update_enabled=false;
    LioEngine engine(config);LioOutput out;bool constrained=false;
    for(int i=0;i<80;++i) {
        auto frame=scan(i*0.1);
        if(i>=12)for(auto& s:frame.imu)s.acceleration[0]=0.1;
        out=engine.process(frame);
        constrained|=out.status.reason.find("stationary velocity constraint")!=std::string::npos;
        if(i>=12)VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    }
    VISTA_CHECK(!constrained);VISTA_CHECK_NEAR(out.status.position_m[0],0,0.04);
    VISTA_CHECK(engine.statistics().imu_updates>100);
    VISTA_CHECK(engine.statistics().lidar_updates>100);
}

VISTA_TEST(lio_reinitialization_retains_accepted_world_anchor_and_local_map) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    config.initial_position_m={5,-3,1};LioEngine engine(config);LioOutput out;
    for(int i=0;i<12;++i)out=engine.process(scan(i*0.1));
    VISTA_CHECK(out.status.pose_valid);const auto accepted=out.status;
    out=engine.process(scan(0));VISTA_CHECK(out.status.state==vista::models::LocalizationState::lost);
    const auto resetting=engine.restart_initialization();
    VISTA_CHECK(resetting.state==vista::models::LocalizationState::initializing && !resetting.pose_valid);
    VISTA_CHECK(resetting.local_map_points==accepted.local_map_points);
    for(int i=0;i<3;++i)VISTA_CHECK_NEAR(resetting.position_m[i],accepted.position_m[i],1e-6);
    for(int i=0;i<12;++i)out=engine.process(scan(10+i*0.1));
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK_NEAR(out.status.position_m[0],5,0.03);
    VISTA_CHECK_NEAR(out.status.position_m[1],-3,0.03);
    VISTA_CHECK(out.status.local_map_points>=accepted.local_map_points);
}

VISTA_TEST(lio_reinitialization_cannot_seed_an_unmatched_room_into_existing_map) {
    LioConfig config;config.initialization_samples=20;config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<12;++i)out=engine.process(scan(i*0.1));
    VISTA_CHECK(out.status.pose_valid);const auto accepted=out.status;
    engine.restart_initialization();
    for(int i=0;i<25;++i) {
        auto frame=scan(10+i*0.1);for(auto& p:frame.cloud.points)p.x+=20;
        out=engine.process(frame);VISTA_CHECK(!out.world_cloud && !out.status.pose_valid);
        VISTA_CHECK(out.status.local_map_points==accepted.local_map_points);
        for(int axis=0;axis<3;++axis)VISTA_CHECK_NEAR(out.status.position_m[axis],accepted.position_m[axis],1e-6);
    }
    VISTA_CHECK(out.status.reason.find("insufficient LiDAR correspondences")!=std::string::npos);
}

VISTA_TEST(lio_geometry_failures_are_timed_and_reported_without_map_publication) {
    LioConfig config;config.initialization_samples=20;LioEngine engine(config);LioOutput out;
    for(int i=0;i<15;++i)out=engine.process(scan(i*0.1,false));
    VISTA_CHECK(!out.status.pose_valid && !out.world_cloud);
    VISTA_CHECK(out.status.state==vista::models::LocalizationState::degraded);
    VISTA_CHECK(out.status.reason.find("degenerate LiDAR geometry")!=std::string::npos);
    VISTA_CHECK(out.status.reason.find("normal_eigen_min=")!=std::string::npos);
    for(int i=15;i<35;++i)out=engine.process(scan(i*0.1,false));
    VISTA_CHECK(out.status.state==vista::models::LocalizationState::lost && !out.world_cloud);
}

VISTA_TEST(lio_worker_reinitializes_after_five_seconds_without_resetting_live_branch) {
    vista::platform::MessageBus bus(64);vista::platform::StopToken stop;
    auto cloud=bus.publisher<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_decoded);
    auto telemetry=bus.subscribe<vista::models::LocalizationStatus>(vista::models::topics::localization_status,"recovery-status");
    auto world=bus.subscribe<vista::devices::LidarPointCloudMessage>(vista::models::topics::pointcloud_world,"recovery-world");
    LioConfig config;config.synchronization_timeout=std::chrono::milliseconds(20);
    std::string error;auto worker=spawn_lio_worker(bus,{"recovery-test",3},stop,config,[&](auto,auto e){error=e;});
    auto send=[&](std::uint64_t seq,std::uint64_t source) {
        auto frame=scan(0);for(auto& p:frame.cloud.points)p.timestamp_ns=source;
        frame.cloud.timestamp_ns=source;
        cloud.publish(vista::devices::LidarPointCloudMessage("lidar",seq,source,100,std::move(frame.cloud),10'000'000'000+seq*100'000'000,10'000'000'000+seq*100'000'000));
    };
    send(0,1'000'000'000);send(1,900'000'000);
    bool lost=false,restarted=false;std::shared_ptr<const vista::models::LocalizationStatus> state;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(8);
    while(std::chrono::steady_clock::now()<deadline) {
        if(telemetry.receive_for(state,std::chrono::milliseconds(500))!=vista::platform::ReceiveStatus::message)continue;
        if(state->state==vista::models::LocalizationState::lost)lost=true;
        if(lost && state->state==vista::models::LocalizationState::initializing && state->reason.find("automatic IMU reinitialization")!=std::string::npos){
            restarted=state->reason.find("previous_loss=LiDAR source changed or sensor timestamp reset/replay loop")!=std::string::npos &&
                state->reason.find("reinitializations=1")!=std::string::npos;break;
        }
    }
    std::shared_ptr<const vista::devices::LidarPointCloudMessage> message;
    const bool published=world.try_receive(message)==vista::platform::ReceiveStatus::message;
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(lost && restarted && !published && error.empty());
}

VISTA_TEST(lio_overlapping_unitree_scans_keep_tracking_without_pose_extrapolation) {
    // Normalization must not change scan/IMU time-bracket semantics.
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

VISTA_TEST(lio_stationary_acceleration_scale_does_not_create_velocity_or_position) {
    for(double magnitude:{7.5,9.81,10.21123684,12.0}) for(double roll:{0.0,180.0}) {
        LioConfig config;config.initialization_samples=20;
        config.initial_position_m={1,2,3};config.initial_rpy_deg={roll,0,0};
        LioEngine engine(config);LioOutput out;
        for(int i=0;i<22;++i) {
            auto frame=scan(i*0.1);frame.cloud.points.clear();
            // Isolate prediction: no LiDAR match can hide IMU integration drift.
            for(auto& sample:frame.imu) sample.acceleration={0,0,magnitude};
            out=engine.process(frame);
            VISTA_CHECK_NEAR(frame.imu.front().acceleration[2],magnitude,1e-12);
        }
        VISTA_CHECK(!out.status.pose_valid && !out.world_cloud);
        for(std::size_t axis=0;axis<3;++axis)
            VISTA_CHECK_NEAR(out.status.position_m[axis],config.initial_position_m[axis],0.005);
    }
}

VISTA_TEST(lio_stationary_room_tracks_with_realistic_unitree_acceleration_scale) {
    LioConfig config;config.initialization_samples=20;
    config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    config.initial_position_m={1,2,0};
    LioEngine engine(config);LioOutput out;
    for(int i=0;i<18;++i) {
        auto frame=scan(i*0.1);
        for(auto& sample:frame.imu) sample.acceleration[2]=10.21123684;
        out=engine.process(frame);
        if(i>=8) VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    }
    for(std::size_t axis=0;axis<3;++axis)
        VISTA_CHECK_NEAR(out.status.position_m[axis],config.initial_position_m[axis],0.03);
}

VISTA_TEST(lio_normalized_acceleration_keeps_real_translation_and_deskew) {
    LioConfig config;config.initialization_samples=20;
    config.scan_voxel_m=0.10;config.map_voxel_m=0.10;
    LioEngine engine(config);LioOutput out;
    auto position=[](double t){const double u=std::max(0.0,t-0.7);return u<0.4?0.25*u*u:0.04+0.2*(u-0.4);};
    for(int i=0;i<20;++i) {
        auto frame=scan(i*0.1);
        for(auto& sample:frame.imu) {
            sample.acceleration[0]=(sample.time_s>=0.7 && sample.time_s<1.1)?0.5:0;
            for(auto& value:sample.acceleration) value*=10.21123684/9.81;
        }
        for(auto& p:frame.cloud.points) {
            const double t=frame.begin_s+static_cast<double>(p.timestamp_ns-frame.cloud.timestamp_ns)*1e-9;
            p.x-=static_cast<float>(position(t));
        }
        out=engine.process(frame);
    }
    VISTA_CHECK(out.status.pose_valid && out.world_cloud);
    VISTA_CHECK_NEAR(out.status.position_m[0],position(2.0),0.06);
    VISTA_CHECK_NEAR(out.status.position_m[1],0,0.04);
    VISTA_CHECK_NEAR(out.status.position_m[2],0,0.04);
    for(const auto& p:out.world_cloud->points) VISTA_CHECK(p.ray_origin_world_m.has_value());
}

VISTA_TEST(lio_coverage_diagnostics_distinguish_missing_left_right_and_gaps) {
    LioSynchronizer sync(0.1);
    VISTA_CHECK(sync.describe_coverage(0,0.1).find("too few IMU samples")!=std::string::npos);
    VISTA_CHECK(sync.add({0.02,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.add({0.04,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.describe_coverage(0,0.03).find("missing left IMU bracket")!=std::string::npos);
    VISTA_CHECK(sync.describe_coverage(0.02,0.1).find("missing right IMU bracket")!=std::string::npos);
    const auto size=sync.size();
    VISTA_CHECK(sync.describe_coverage(0.02,0.04).find("covered;")==0);
    VISTA_CHECK(sync.size()==size && sync.take(0.02,0.04));
    VISTA_CHECK(sync.add({0.2,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.describe_coverage(0.04,0.2).find("IMU gap exceeds limit")!=std::string::npos);
    VISTA_CHECK(!sync.take(0.04,0.2));
    VISTA_CHECK(sync.describe_coverage(0.04,0.04).find("covered;")==0);
    VISTA_CHECK(sync.describe_coverage(1,0).find("invalid interval")!=std::string::npos);
}

VISTA_TEST(lio_coverage_diagnostics_track_capacity_evictions_not_normal_pruning) {
    LioSynchronizer sync(0.1);
    for(int i=0;i<6000;++i) VISTA_CHECK(sync.add({i*0.002,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.size()==4096 && sync.evicted_samples()==1904);
    const auto diagnostic=sync.describe_coverage(0,0.1);
    VISTA_CHECK(diagnostic.find("missing left IMU bracket")!=std::string::npos);
    VISTA_CHECK(diagnostic.find("imu_buffer_evictions=1904")!=std::string::npos);
    VISTA_CHECK(sync.take(11.8,11.9));
    VISTA_CHECK(sync.evicted_samples()==1904);
    sync.clear();VISTA_CHECK(sync.size()==0 && sync.evicted_samples()==0);
}

VISTA_TEST(lio_gap_diagnostics_remain_a_safe_grafana_string_field) {
    LioSynchronizer sync(0.1);
    VISTA_CHECK(sync.add({0,{0,0,9.81},{0,0,0}}));
    VISTA_CHECK(sync.add({0.02,{0,0,9.81},{0,0,0}}));
    vista::models::LocalizationStatus state;
    state.state=vista::models::LocalizationState::lost;
    state.reason="IMU coverage lost: "+sync.describe_coverage(0,0.1)+
        "; imu_topic_drops=7; cloud_topic_drops=2; pending_scan_drops=3; restart required";
    const auto line=format_localization_measurement(state);
    VISTA_CHECK(line.find("pose_valid=0i")!=std::string::npos);
    VISTA_CHECK(line.find("missing right IMU bracket")!=std::string::npos);
    VISTA_CHECK(line.find("imu_topic_drops=7")!=std::string::npos);
    VISTA_CHECK(line.find('\n')==std::string::npos);
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
