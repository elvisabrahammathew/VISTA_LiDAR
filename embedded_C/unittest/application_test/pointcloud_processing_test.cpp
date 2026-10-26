#include <limits>
#include <optional>
#include <vector>

#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include "unittest/test.hpp"
#include "models/topics.hpp"

VISTA_TEST(pointcloud_cleaning_sets_world_ray_origin_without_ground_removal) {
    vista::application::PreprocessingConfig config;
    config.voxel_size_m.reset();
    vista::application::MountingConfig mount;
    mount.mount_x_m=-10.0F; mount.mount_y_m=2.0F; mount.mount_z_m=0.75F;
    config.mounting=mount;
    vista::application::PointCloudPreprocessor processor(config);
    const auto cloud=processor.clean({1,{{1,0,0,0,0,0,0}}});
    VISTA_CHECK(cloud.sensor_origin_world_m.has_value());
    VISTA_CHECK_NEAR((*cloud.sensor_origin_world_m)[0],-10.0F,1.0e-6F);
    VISTA_CHECK_NEAR((*cloud.sensor_origin_world_m)[1],2.0F,1.0e-6F);
    VISTA_CHECK_NEAR((*cloud.sensor_origin_world_m)[2],0.75F,1.0e-6F);
    VISTA_CHECK_NEAR(cloud.points[0].x,-9.0F,1.0e-6F);
    const auto processed=processor.process_ground(cloud);
    VISTA_CHECK(processed.sensor_origin_world_m==cloud.sensor_origin_world_m);
}

namespace {

vista::models::PointXYZIRT point(float x, float y, float z) {
    return {x, y, z, 1, 0, 0, 10};
}

vista::models::PointCloudFrame frame(
    std::vector<vista::models::PointXYZIRT> points) {
    return {10, std::move(points)};
}

}  // namespace

VISTA_TEST(preprocessing_filters_range_roi_and_invalid_points) {
    vista::application::PreprocessingConfig config{
        1.0F,
        10.0F,
        vista::application::AxisAlignedRoi{0.0F, 10.0F, -2.0F, 2.0F, -2.0F, 2.0F},
        std::nullopt,
        std::nullopt,
    };
    const auto output = vista::application::preprocess_point_cloud(
        frame({
            point(2.0F, 0.0F, 0.0F),
            point(0.1F, 0.0F, 0.0F),
            point(20.0F, 0.0F, 0.0F),
            point(2.0F, 3.0F, 0.0F),
            point(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F),
        }),
        config);
    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK(output.points[0] == point(2.0F, 0.0F, 0.0F));
}

VISTA_TEST(preprocessing_keeps_first_point_in_each_voxel) {
    vista::application::PreprocessingConfig config{
        0.0F, 10.0F, std::nullopt, 1.0F, std::nullopt};
    const auto output = vista::application::preprocess_point_cloud(
        frame({
            point(1.1F, 2.1F, 3.1F),
            point(1.8F, 2.8F, 3.8F),
            point(2.1F, 2.1F, 3.1F),
        }),
        config);
    VISTA_CHECK(output.points.size() == 2);
    VISTA_CHECK(output.points[0] == point(1.1F, 2.1F, 3.1F));
    VISTA_CHECK(output.points[1] == point(2.1F, 2.1F, 3.1F));
}

VISTA_TEST(preprocessing_removes_ground_band) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::static_height;
    ground.floor_z_m = -1.5F;
    ground.distance_threshold_m = 0.15F;
    vista::application::PreprocessingConfig config{
        0.0F,
        10.0F,
        std::nullopt,
        std::nullopt,
        ground,
    };
    const auto output = vista::application::preprocess_point_cloud(
        frame({
            point(2.0F, 0.0F, -1.50F),
            point(2.0F, 0.0F, -1.40F),
            point(2.0F, 0.0F, -0.50F),
        }),
        config);
    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK(output.points[0] == point(2.0F, 0.0F, -0.50F));
}

VISTA_TEST(preprocessing_transforms_upright_table_mount_to_world_frame) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::static_height;
    ground.mount_z_m = 0.75F;
    ground.floor_z_m = 0.0F;
    ground.distance_threshold_m = 0.05F;
    vista::application::PreprocessingConfig config{
        0.0F, 10.0F, std::nullopt, std::nullopt, ground};

    const auto output = vista::application::preprocess_point_cloud(
        frame({point(1.0F, 0.0F, -0.75F), point(1.0F, 0.0F, 0.25F)}),
        config);

    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK_NEAR(output.points[0].z, 1.0F, 1.0e-5F);
}

VISTA_TEST(preprocessing_transforms_inverted_ceiling_mount_to_world_frame) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::static_height;
    ground.mount_z_m = 2.5F;
    ground.mount_roll_deg = 180.0F;
    ground.floor_z_m = 0.0F;
    ground.distance_threshold_m = 0.05F;
    vista::application::PreprocessingConfig config{
        0.0F, 10.0F, std::nullopt, std::nullopt, ground};

    const auto output = vista::application::preprocess_point_cloud(
        frame({point(1.0F, 0.0F, 2.5F), point(1.0F, 0.0F, 1.5F)}),
        config);

    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK_NEAR(output.points[0].z, 1.0F, 1.0e-5F);
}

VISTA_TEST(preprocessing_imu_setting_falls_back_to_mount_pose_without_imu) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::static_height;
    ground.use_imu = true;
    ground.mount_z_m = 2.5F;
    ground.mount_roll_deg = 180.0F;
    ground.distance_threshold_m = 0.05F;
    vista::application::PointCloudPreprocessor processor({
        0.0F, 10.0F, std::nullopt, std::nullopt, ground});

    const auto output = processor.process(
        frame({point(1.0F, 0.0F, 2.5F), point(1.0F, 0.0F, 1.5F)}));

    VISTA_CHECK(!processor.using_imu_orientation());
    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK_NEAR(output.points[0].z, 1.0F, 1.0e-5F);
}

VISTA_TEST(preprocessing_uses_stationary_imu_gravity_when_available) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::static_height;
    ground.use_imu = true;
    ground.mount_z_m = 0.75F;
    ground.mount_roll_deg = 180.0F;  // Deliberately wrong; IMU overrides tilt.
    ground.distance_threshold_m = 0.05F;
    vista::application::PointCloudPreprocessor processor({
        0.0F, 10.0F, std::nullopt, std::nullopt, ground});
    vista::models::ImuFrame imu;
    imu.linear_acceleration_z_m_s2 = 9.81F;
    processor.update_imu(imu);

    const auto output = processor.process(
        frame({point(1.0F, 0.0F, -0.75F), point(1.0F, 0.0F, 0.25F)}));

    VISTA_CHECK(processor.using_imu_orientation());
    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK_NEAR(output.points[0].z, 1.0F, 1.0e-5F);
}

VISTA_TEST(preprocessing_ransac_calibrates_once_after_requested_frames) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::ransac;
    ground.distance_threshold_m = 0.03F;
    ground.normal_tolerance_deg = 10.0F;
    ground.calibration_frames = 2;
    ground.minimum_inlier_ratio = 0.5F;
    vista::application::PointCloudPreprocessor processor({
        0.0F, 20.0F, std::nullopt, std::nullopt, ground});
    const std::vector<vista::models::PointXYZIRT> sample{
        point(-2.0F, -2.0F, 0.0F), point(-2.0F, 0.0F, 0.0F),
        point(-2.0F, 2.0F, 0.0F), point(0.0F, -2.0F, 0.0F),
        point(0.0F, 0.0F, 0.0F), point(0.0F, 2.0F, 0.0F),
        point(2.0F, -2.0F, 0.0F), point(2.0F, 0.0F, 0.0F),
        point(2.0F, 2.0F, 0.0F), point(1.0F, 1.0F, 1.0F),
    };

    const auto first = processor.process(frame(sample));
    const auto second = processor.process(frame(sample));

    VISTA_CHECK(first.points.size() == sample.size());
    VISTA_CHECK(processor.ground_calibrated());
    VISTA_CHECK(second.points.size() == 1);
    VISTA_CHECK_NEAR(second.points[0].z, 1.0F, 1.0e-5F);
}

VISTA_TEST(preprocessing_hybrid_uses_static_floor_during_calibration) {
    vista::application::GroundRemovalConfig ground;
    ground.mode = vista::application::GroundMode::hybrid;
    ground.distance_threshold_m = 0.05F;
    ground.calibration_frames = 50;
    vista::application::PointCloudPreprocessor processor({
        0.0F, 10.0F, std::nullopt, std::nullopt, ground});

    const auto output = processor.process(
        frame({point(1.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 1.0F)}));

    VISTA_CHECK(!processor.ground_calibrated());
    VISTA_CHECK(output.points.size() == 1);
    VISTA_CHECK_NEAR(output.points[0].z, 1.0F, 1.0e-5F);
    const auto status=processor.ground_status();
    VISTA_CHECK(status.state==vista::models::GroundState::calibrating);
    VISTA_CHECK(status.using_static_fallback);
    VISTA_CHECK(status.input_point_count==2);
    VISTA_CHECK(status.removed_ground_point_count==1);
    VISTA_CHECK_NEAR(status.removed_ground_ratio,0.5F,1.0e-5F);
}

VISTA_TEST(preprocessing_rejects_invalid_voxel_size) {
    vista::application::PreprocessingConfig config;
    config.voxel_size_m = 0.0F;
    VISTA_CHECK_THROWS(vista::application::preprocess_point_cloud(frame({}), config));
}

VISTA_TEST(cleaned_cloud_preserves_ground_and_ground_stage_does_not_transform_twice) {
    vista::application::GroundRemovalConfig mount;
    mount.mount_z_m=0.75F; mount.distance_threshold_m=0.05F;
    vista::application::PointCloudPreprocessor processor({
        0.0F,10.0F,std::nullopt,std::nullopt,mount,mount});
    const auto cleaned=processor.clean(frame({point(1,0,-0.75F),point(1,0,0.25F)}));
    const auto processed=processor.process_ground(cleaned);
    VISTA_CHECK(cleaned.points.size()==2);
    VISTA_CHECK_NEAR(cleaned.points[0].z,0.0F,1.0e-5F);
    VISTA_CHECK(processed.points.size()==1);
    VISTA_CHECK_NEAR(processed.points[0].z,1.0F,1.0e-5F);
}

VISTA_TEST(mounting_is_applied_when_ground_removal_is_disabled) {
    vista::application::MountingConfig mount; mount.mount_z_m=2.5F;
    vista::application::PointCloudPreprocessor processor({
        0.0F,10.0F,std::nullopt,std::nullopt,std::nullopt,mount});
    const auto cleaned=processor.clean(frame({point(1,0,-2.5F)}));
    VISTA_CHECK(cleaned.points.size()==1);
    VISTA_CHECK_NEAR(cleaned.points[0].z,0.0F,1.0e-5F);
}

VISTA_TEST(runtime_cleaning_has_no_ground_or_imu_subscription_and_preserves_floor) {
    using namespace vista;
    platform::MessageBus bus(8);platform::StopToken stop;
    // Incompatible types fail synchronously if a retired subscription/publisher
    // is accidentally restored. No real sensor or network endpoint is involved.
    auto no_imu=bus.publisher<int>(models::topics::lidar_imu);
    auto no_ground=bus.publisher<int>(models::topics::ground_status);
    auto no_processed=bus.publisher<int>(models::topics::pointcloud_processed);
    auto input=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_decoded);
    auto output=bus.subscribe<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned,"test-clean-no-ground");
    application::PreprocessingConfig config;config.voxel_size_m.reset();
    application::MountingConfig mount;mount.mount_z_m=.75F;config.mounting=mount;
    // Even a direct legacy API caller cannot accidentally activate it in a worker.
    application::GroundRemovalConfig legacy;legacy.distance_threshold_m=.1F;
    config.ground_removal=legacy;
    std::string error;
    auto worker=application::spawn_preprocessing_worker(bus,{"test-clean-no-ground",3},stop,config,
        [&](auto,auto e){error=std::move(e);});
    input.publish(devices::LidarPointCloudMessage("test",7,10,20,
        models::PointCloudFrame(10,{{1,0,-.75F,1,0,0,10},{1,0,.25F,1,0,0,11}}),30,31,4,29));
    std::shared_ptr<const devices::LidarPointCloudMessage> received;
    const auto result=output.receive_for(received,std::chrono::seconds(2));
    stop.request_stop();bus.close();worker.join();
    VISTA_CHECK(result==platform::ReceiveStatus::message && error.empty());
    VISTA_CHECK(received->payload.points.size()==2);
    VISTA_CHECK_NEAR(received->payload.points[0].z,0,1e-6);
    VISTA_CHECK_NEAR(received->payload.points[1].z,1,1e-6);
    VISTA_CHECK(received->sequence==7 && received->received_monotonic_ns==30 && received->measurement_timestamp_ns==31);
    VISTA_CHECK(received->measurement_clock_epoch==4 && received->measurement_epoch_started_ns==29);
    VISTA_CHECK(received->payload.sensor_origin_world_m && (*received->payload.sensor_origin_world_m)[2]==.75F);
}
