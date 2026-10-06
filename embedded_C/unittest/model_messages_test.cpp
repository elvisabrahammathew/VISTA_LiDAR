#include "models/imu/imu_message.hpp"
#include "models/mapping/ground_status.hpp"
#include "models/mapping/localization.hpp"
#include "models/mapping/room_map.hpp"
#include <chrono>
#include <type_traits>
#include "1_Platform/message_bus/message_bus.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "4_Applications/grafana_bridge/grafana_bridge.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"

namespace {
using namespace vista;
static_assert(!std::is_same_v<models::ImuMessage,models::LidarMessage<models::ImuFrame>>);
static_assert(!std::is_same_v<models::GroundStatusMessage,models::LidarMessage<models::GroundStatus>>);
static_assert(!std::is_same_v<models::RoomMapPoint,models::PointXYZIRT>);
static_assert(!std::is_same_v<models::RoomMapFrame,models::PointCloudFrame>);
static_assert(!std::is_convertible_v<models::ImuMessage,models::LidarMessage<models::ImuFrame>>);
static_assert(!std::is_convertible_v<models::PointCloudFrame,models::RoomMapFrame>);
}

VISTA_TEST(model_topics_reject_reusing_lidar_envelopes_for_imu_ground_and_map) {
    platform::MessageBus bus;
    (void)bus.publisher<models::ImuMessage>(models::topics::lidar_imu);
    (void)bus.publisher<models::GroundStatusMessage>(models::topics::ground_status);
    (void)bus.publisher<models::RoomMapMessage>(models::topics::room_map);
    VISTA_CHECK_THROWS(bus.publisher<models::LidarMessage<models::ImuFrame>>(models::topics::lidar_imu));
    VISTA_CHECK_THROWS(bus.publisher<models::LidarMessage<models::GroundStatus>>(models::topics::ground_status));
    VISTA_CHECK_THROWS(bus.publisher<devices::LidarPointCloudMessage>(models::topics::room_map));
}

VISTA_TEST(independent_imu_envelope_keeps_source_and_host_clock_metadata) {
    models::ImuFrame sample;sample.timestamp_ns=5'000'000;sample.linear_acceleration_z_m_s2=9.81F;
    platform::MessageBus bus;
    auto input=bus.subscribe<models::ImuMessage>(models::topics::lidar_imu,"test-independent-imu");
    auto output=bus.publisher<models::ImuMessage>(models::topics::lidar_imu);
    output.publish({"external-imu",7,sample.timestamp_ns,123,sample,10'000'000'000,9'999'000'000});
    std::shared_ptr<const models::ImuMessage> received;
    VISTA_CHECK(input.receive_for(received,std::chrono::milliseconds(100))==platform::ReceiveStatus::message);
    VISTA_CHECK(received->source_id=="external-imu" && received->sequence==7);
    VISTA_CHECK(received->received_monotonic_ns==10'000'000'000);
    VISTA_CHECK(received->measurement_timestamp_ns==9'999'000'000ULL);
    VISTA_CHECK(received->received_timestamp_ns==123 && received->sensor_timestamp_ns==sample.timestamp_ns);
    VISTA_CHECK_NEAR(received->payload.linear_acceleration_z_m_s2,9.81F,1e-6F);
}

VISTA_TEST(independent_ground_envelope_keeps_grafana_labels_and_timing) {
    models::GroundStatus status;status.timestamp_ns=100;status.configured_mode="hybrid";
    status.state=models::GroundState::static_fallback;
    models::GroundStatusMessage message("unitree-l2",3,9,100,status,1000,900);
    VISTA_CHECK(message.source_id=="unitree-l2" && message.received_monotonic_ns==1000);
    VISTA_CHECK(message.measurement_timestamp_ns==900ULL);
    const auto line=application::format_ground_measurement(message);
    VISTA_CHECK(line.find("ground_status,lidar_id=unitree-l2,mode=hybrid ")==0);
    VISTA_CHECK(application::format_ground_state_measurement(message).find("state_code=2i")!=std::string::npos);
}

VISTA_TEST(independent_room_map_encoder_preserves_lpc1_bytes_without_lidar_envelope) {
    models::RoomMapMessage map(17,200,{100,{{1,2,3,8},{4,5,6,9}}});
    devices::LidarPointCloudMessage legacy("room-map",17,std::nullopt,200,
        {100,{{1,2,3,8,2,1,77},{4,5,6,9,4,2,88}}});
    VISTA_CHECK(application::encode_lpc1_pointcloud(map,100)==application::encode_lpc1_pointcloud(legacy,100));
    VISTA_CHECK(application::encode_lpc1_pointcloud(map,1)==application::encode_lpc1_pointcloud(legacy,1));
    map.payload.timestamp_ns=0;legacy.payload.timestamp_ns=0;
    VISTA_CHECK(application::encode_lpc1_pointcloud(map,100)==application::encode_lpc1_pointcloud(legacy,100));
    VISTA_CHECK_THROWS(application::encode_lpc1_pointcloud(map,0));
    models::RoomMapMessage empty(18,300,{});
    VISTA_CHECK(application::encode_lpc1_pointcloud(empty,1).size()==32);
}
