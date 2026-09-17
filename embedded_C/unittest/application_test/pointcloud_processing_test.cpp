#include <limits>
#include <optional>
#include <vector>

#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include "unittest/test.hpp"

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
    vista::application::PreprocessingConfig config{
        0.0F,
        10.0F,
        std::nullopt,
        std::nullopt,
        vista::application::GroundRemovalConfig{-1.5F, 0.15F},
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

VISTA_TEST(preprocessing_rejects_invalid_voxel_size) {
    vista::application::PreprocessingConfig config;
    config.voxel_size_m = 0.0F;
    VISTA_CHECK_THROWS(vista::application::preprocess_point_cloud(frame({}), config));
}
