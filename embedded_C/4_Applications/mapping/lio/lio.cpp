#include "4_Applications/mapping/lio/lio_adapter.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/topics.hpp"
#include "1_Platform/timing/measurement_clock.hpp"
#include <algorithm>
#include <deque>
#include <limits>

namespace vista::application {
platform::WorkerHandle spawn_lio_worker(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,LioConfig config,LioCompletion on_complete) {
    validate_lio_config(config);
    platform::WorkerTopicInputs inputs(thread.name);
    auto cloud=inputs.subscribe<devices::LidarPointCloudMessage>(bus,models::topics::pointcloud_decoded);
    auto imu=inputs.subscribe<models::ImuMessage>(bus,models::topics::lidar_imu);
    auto world=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_world);
    auto telemetry=bus.publisher<models::LocalizationStatus>(models::topics::localization_status);
    return platform::spawn_worker(std::move(thread),[stop,config,inputs=std::move(inputs),cloud=std::move(cloud),
        imu=std::move(imu),world=std::move(world),telemetry=std::move(telemetry),on_complete=std::move(on_complete)]() mutable {
        try {
            LioEngine engine(config);LioSynchronizer sync(config.maximum_imu_gap_s);
            LioReport report;models::LocalizationStatus latest;
            std::uint64_t reference_host{};bool have_reference{},clock_fault{},cloud_closed{};
            std::optional<std::uint64_t> last_imu_source_time, last_imu_host_time, last_cloud_source_time;
            std::string imu_id, lidar_id;
            std::string fault_reason;
            auto fail=[&](const char* reason) { if(!clock_fault) fault_reason=reason;clock_fault=true; };
            std::optional<double> previous_end;
            std::deque<models::ImuMessage> early_imu;
            struct Pending { std::shared_ptr<const devices::LidarPointCloudMessage> message;double begin,end;std::chrono::steady_clock::time_point arrival; };
            std::deque<Pending> pending;
            auto heartbeat=std::chrono::steady_clock::now();auto last_valid=heartbeat;
            auto accept_imu=[&](models::ImuMessage message) {
                if(clock_fault)return;
                if(!have_reference){early_imu.push_back(std::move(message));while(early_imu.size()>4096)early_imu.pop_front();return;}
                // One active producer, no configured source selector. Refuse to
                // mix two physical IMUs or silently switch an epoch mid-session.
                if(message.lidar_id.empty() || !message.measurement_timestamp_ns || message.received_monotonic_ns==0)
                    {fail("IMU missing normalized host measurement time");return;}
                if(!imu_id.empty() && message.lidar_id!=imu_id)
                    {fail("multiple IMU sources or source changed");return;}
                if(last_imu_host_time && *message.measurement_timestamp_ns<=*last_imu_host_time)
                    {fail("IMU normalized host time is not monotonic");return;}
                if(message.payload.timestamp_ns && last_imu_source_time && message.payload.timestamp_ns<=*last_imu_source_time)
                    {fail("IMU sensor timestamp reset/replay loop");return;}
                imu_id=message.lidar_id;
                last_imu_host_time=message.measurement_timestamp_ns;
                if(message.payload.timestamp_ns)last_imu_source_time=message.payload.timestamp_ns;
                const auto t=static_cast<double>((static_cast<long double>(*message.measurement_timestamp_ns)-reference_host)*1e-9L);
                const auto& p=message.payload;
                if(!sync.add({t,{p.linear_acceleration_x_m_s2,p.linear_acceleration_y_m_s2,p.linear_acceleration_z_m_s2},
                                {p.angular_velocity_x_rad_s,p.angular_velocity_y_rad_s,p.angular_velocity_z_rad_s}})) fail("invalid IMU values or time ordering");
            };
            while(!stop.is_stop_requested() && !cloud_closed) {
                const auto ready=inputs.wait_for(std::chrono::milliseconds(20));
                if(imu.is_ready(ready)) {
                    std::shared_ptr<const models::ImuMessage> m;
                    while(imu.try_receive(m)==platform::ReceiveStatus::message) accept_imu(*m);
                }
                if(cloud.is_ready(ready)) {
                    std::shared_ptr<const devices::LidarPointCloudMessage> m;
                    while(true) {
                        const auto received=cloud.try_receive(m);
                        if(received==platform::ReceiveStatus::closed){cloud_closed=true;break;}
                        if(received!=platform::ReceiveStatus::message) break;
                        if(m->payload.points.empty()) continue;
                        ++report.received;
                        auto minimum=std::numeric_limits<std::uint64_t>::max();std::uint64_t maximum{};
                        for(const auto& p:m->payload.points){minimum=std::min(minimum,p.timestamp_ns);maximum=std::max(maximum,p.timestamp_ns);}
                        if(minimum==0 || maximum<minimum || maximum-minimum>500'000'000ULL){++report.rejected;latest.reason="invalid per-point timestamps/scan duration";latest.state=models::LocalizationState::degraded;latest.pose_valid=false;continue;}
                        const auto duration_ns=maximum-minimum;
                        if(m->lidar_id.empty() || m->received_monotonic_ns==0 || !m->sensor_timestamp_ns ||
                           (!lidar_id.empty() && lidar_id!=m->lidar_id) ||
                           (last_cloud_source_time && minimum<=*last_cloud_source_time)) {fail("LiDAR source changed or sensor timestamp reset/replay loop");continue;}
                        const auto packet_reference=*m->sensor_timestamp_ns;
                        const auto reference_distance=packet_reference>minimum?packet_reference-minimum:minimum-packet_reference;
                        if(reference_distance>500'000'000ULL){fail("LiDAR packet/point timestamps disagree");continue;}
                        const auto host_begin=platform::shift_measurement_time(m->measurement_timestamp_ns,minimum,*m->sensor_timestamp_ns);
                        if(!host_begin){fail("LiDAR missing normalized host measurement time");continue;}
                        lidar_id=m->lidar_id;last_cloud_source_time=minimum;
                        if(!have_reference){reference_host=*host_begin;have_reference=true;for(auto& sample:early_imu)accept_imu(std::move(sample));early_imu.clear();}
                        const auto start=static_cast<double>((static_cast<long double>(*host_begin)-reference_host)*1e-9L);
                        if(pending.size()==4){pending.pop_front();++report.dropped;}
                        pending.push_back({m,start,start+static_cast<double>(duration_ns)*1e-9,std::chrono::steady_clock::now()});
                    }
                }
                if(clock_fault) {
                    // Never restart the world frame silently: that would corrupt
                    // the persistent room map built with the old origin.
                    pending.clear();latest.state=models::LocalizationState::lost;latest.pose_valid=false;
                    latest.reason=fault_reason+"; restart required";
                } else while(!pending.empty()) {
                    auto& p=pending.front();
                    const auto batch=sync.take(previous_end?std::min(*previous_end,p.begin):p.begin,p.end);
                    if(!batch) {
                        if(std::chrono::steady_clock::now()-p.arrival<config.synchronization_timeout) break;
                        ++report.rejected;latest.pose_valid=false;
                        if(previous_end) {
                            // An uncovered propagation interval cannot safely be
                            // skipped or reseeded in a persistent world map.
                            fail("IMU coverage lost after LIO start (gap/overflow/alignment)");
                            latest.state=models::LocalizationState::lost;
                            latest.reason=fault_reason+"; restart required";
                            pending.clear();break;
                        }
                        latest.state=models::LocalizationState::waiting_imu;
                        latest.reason="IMU missing, stale, gapped, or arrival-based alignment insufficient";pending.pop_front();continue;
                    }
                    LioScan scan;scan.cloud=p.message->payload;
                    scan.cloud.timestamp_ns=std::min_element(scan.cloud.points.begin(),scan.cloud.points.end(),[](const auto& a,const auto& b){return a.timestamp_ns<b.timestamp_ns;})->timestamp_ns;
                    scan.begin_s=p.begin;scan.end_s=p.end;scan.imu=*batch;
                    auto output=engine.process(scan);latest=std::move(output.status);previous_end=p.end;
                    if(output.world_cloud){world.publish(devices::LidarPointCloudMessage(p.message->lidar_id,p.message->sequence,p.message->sensor_timestamp_ns,p.message->received_timestamp_ns,std::move(*output.world_cloud),p.message->received_monotonic_ns,p.message->measurement_timestamp_ns));++report.published;last_valid=std::chrono::steady_clock::now();}
                    else ++report.rejected;
                    pending.pop_front();
                }
                const auto now=std::chrono::steady_clock::now();
                if(now>=heartbeat) {
                    if(latest.pose_valid && now-last_valid>config.synchronization_timeout){latest.pose_valid=false;latest.state=models::LocalizationState::degraded;latest.reason="no fresh localized scan (input/processing delay); map held";}
                    latest.timestamp_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                    latest.dropped_messages=report.dropped+cloud.dropped_messages()+imu.dropped_messages();
                    latest.rejected_scans=report.rejected;
                    telemetry.publish(latest);heartbeat=now+std::chrono::seconds(1);
                }
            }
            report.dropped+=cloud.dropped_messages()+imu.dropped_messages();
            on_complete(report,{});
        }catch(const std::exception& error){models::LocalizationStatus status;status.state=models::LocalizationState::lost;status.reason=error.what();telemetry.publish(status);on_complete(std::nullopt,error.what());}
    });
}
} // namespace vista::application
