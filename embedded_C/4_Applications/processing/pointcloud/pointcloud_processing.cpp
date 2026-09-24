#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "models/topics.hpp"

namespace vista::application {
namespace {

constexpr float pi = 3.14159265358979323846F;
constexpr std::size_t maximum_calibration_samples = 100'000;
constexpr std::size_t maximum_samples_per_frame = 2'000;
constexpr std::size_t ransac_iterations = 256;

struct Vec3 { float x{}; float y{}; float z{}; };
struct Matrix3 { float value[3][3]{}; };
struct Plane { Vec3 normal{0.0F, 0.0F, 1.0F}; float d{}; };
struct PlaneEstimate {
    Plane plane;
    std::size_t inlier_count{};
    float inlier_ratio{};
    float mean_residual_m{};
    float rms_residual_m{};
    float p95_residual_m{};
};

struct VoxelKey {
    int x{}; int y{}; int z{};
    bool operator==(const VoxelKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& value) const noexcept {
        auto seed = std::hash<int>{}(value.x);
        seed ^= std::hash<int>{}(value.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<int>{}(value.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

float degrees_to_radians(float degrees) { return degrees * pi / 180.0F; }
float dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
float norm(const Vec3& value) { return std::sqrt(dot(value, value)); }
Vec3 normalized(const Vec3& value) {
    const auto length = norm(value);
    if (!std::isfinite(length) || length <= 1.0e-6F) {
        throw std::invalid_argument("cannot normalize a zero/non-finite vector");
    }
    return {value.x/length, value.y/length, value.z/length};
}

Matrix3 identity_matrix() {
    Matrix3 output{};
    output.value[0][0] = output.value[1][1] = output.value[2][2] = 1.0F;
    return output;
}

Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
    Matrix3 output{};
    for (std::size_t row=0; row<3; ++row) {
        for (std::size_t column=0; column<3; ++column) {
            for (std::size_t index=0; index<3; ++index) {
                output.value[row][column] += a.value[row][index]*b.value[index][column];
            }
        }
    }
    return output;
}

Vec3 multiply(const Matrix3& matrix, const Vec3& vector) {
    return {
        matrix.value[0][0]*vector.x + matrix.value[0][1]*vector.y + matrix.value[0][2]*vector.z,
        matrix.value[1][0]*vector.x + matrix.value[1][1]*vector.y + matrix.value[1][2]*vector.z,
        matrix.value[2][0]*vector.x + matrix.value[2][1]*vector.y + matrix.value[2][2]*vector.z,
    };
}

Matrix3 euler_rotation(float roll_deg, float pitch_deg, float yaw_deg) {
    const auto roll=degrees_to_radians(roll_deg), pitch=degrees_to_radians(pitch_deg), yaw=degrees_to_radians(yaw_deg);
    const auto cr=std::cos(roll), sr=std::sin(roll), cp=std::cos(pitch), sp=std::sin(pitch), cy=std::cos(yaw), sy=std::sin(yaw);
    Matrix3 output{};
    // Rz(yaw) * Ry(pitch) * Rx(roll), sensor frame to world frame.
    output.value[0][0]=cy*cp; output.value[0][1]=cy*sp*sr-sy*cr; output.value[0][2]=cy*sp*cr+sy*sr;
    output.value[1][0]=sy*cp; output.value[1][1]=sy*sp*sr+cy*cr; output.value[1][2]=sy*sp*cr-cy*sr;
    output.value[2][0]=-sp; output.value[2][1]=cp*sr; output.value[2][2]=cp*cr;
    return output;
}

Matrix3 align_vector_to_world_up(const Vec3& sensor_up) {
    const auto from=normalized(sensor_up);
    const Vec3 to{0.0F,0.0F,1.0F};
    const auto cosine=std::clamp(dot(from,to),-1.0F,1.0F);
    if (cosine > 1.0F-1.0e-6F) return identity_matrix();
    if (cosine < -1.0F+1.0e-6F) return euler_rotation(180.0F,0.0F,0.0F);
    const auto axis_cross=cross(from,to);
    const auto sine_squared=dot(axis_cross,axis_cross);
    Matrix3 skew{};
    skew.value[0][1]=-axis_cross.z; skew.value[0][2]=axis_cross.y;
    skew.value[1][0]=axis_cross.z; skew.value[1][2]=-axis_cross.x;
    skew.value[2][0]=-axis_cross.y; skew.value[2][1]=axis_cross.x;
    const auto skew_squared=multiply(skew,skew);
    auto output=identity_matrix();
    const auto scale=(1.0F-cosine)/sine_squared;
    for (std::size_t row=0; row<3; ++row) for (std::size_t column=0; column<3; ++column)
        output.value[row][column] += skew.value[row][column] + skew_squared.value[row][column]*scale;
    return output;
}

Matrix3 imu_rotation(const Vec3& sensor_up, float yaw_deg) {
    // Gravity fixes roll/pitch but not heading, so preserve configured yaw.
    return multiply(euler_rotation(0.0F,0.0F,yaw_deg), align_vector_to_world_up(sensor_up));
}

std::string lower_copy(std::string value) {
    std::transform(value.begin(),value.end(),value.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    return value;
}

bool contains(const AxisAlignedRoi& roi, const models::PointXYZIRT& point) {
    return point.x>=roi.min_x && point.x<=roi.max_x && point.y>=roi.min_y && point.y<=roi.max_y && point.z>=roi.min_z && point.z<=roi.max_z;
}

Plane static_ground_plane(const GroundRemovalConfig& config) {
    return {{0.0F,0.0F,1.0F},-config.floor_z_m};
}

float point_plane_distance(const Plane& plane, const models::PointXYZIRT& point) {
    return std::fabs(plane.normal.x*point.x + plane.normal.y*point.y + plane.normal.z*point.z + plane.d);
}

void validate(const PreprocessingConfig& config) {
    if (!std::isfinite(config.min_distance_m) || !std::isfinite(config.max_distance_m) || config.min_distance_m<0.0F || config.max_distance_m<config.min_distance_m)
        throw std::invalid_argument("point-cloud distance limits must be finite, non-negative, and ordered");
    if (config.region_of_interest) {
        const auto& roi=*config.region_of_interest;
        const float values[]{roi.min_x,roi.max_x,roi.min_y,roi.max_y,roi.min_z,roi.max_z};
        for (const auto value:values) if (!std::isfinite(value)) throw std::invalid_argument("ROI bounds must be finite");
        if (roi.min_x>roi.max_x || roi.min_y>roi.max_y || roi.min_z>roi.max_z) throw std::invalid_argument("every ROI minimum must be <= its maximum");
    }
    if (config.voxel_size_m && (!std::isfinite(*config.voxel_size_m) || *config.voxel_size_m<=0.0F))
        throw std::invalid_argument("voxel size must be positive and finite");
    if (!config.ground_removal) return;
    const auto& ground=*config.ground_removal;
    const float finite_values[]{ground.mount_x_m,ground.mount_y_m,ground.mount_z_m,ground.mount_roll_deg,ground.mount_pitch_deg,ground.mount_yaw_deg,ground.floor_z_m,ground.distance_threshold_m,ground.normal_tolerance_deg,ground.minimum_inlier_ratio};
    if (!std::all_of(std::begin(finite_values),std::end(finite_values),[](float v){return std::isfinite(v);}))
        throw std::invalid_argument("ground calibration values must be finite");
    if (ground.distance_threshold_m<=0.0F) throw std::invalid_argument("ground distance threshold must be positive");
    if (ground.normal_tolerance_deg<=0.0F || ground.normal_tolerance_deg>90.0F) throw std::invalid_argument("ground normal tolerance must be in the range (0, 90]");
    if (ground.calibration_frames==0) throw std::invalid_argument("ground calibration frames must be non-zero");
    if (ground.minimum_inlier_ratio<=0.0F || ground.minimum_inlier_ratio>1.0F) throw std::invalid_argument("ground minimum inlier ratio must be in the range (0, 1]");
}

std::optional<Plane> plane_from_points(const models::PointXYZIRT& first,const models::PointXYZIRT& second,const models::PointXYZIRT& third) {
    const Vec3 a{second.x-first.x,second.y-first.y,second.z-first.z};
    const Vec3 b{third.x-first.x,third.y-first.y,third.z-first.z};
    auto normal=cross(a,b); const auto length=norm(normal);
    if (!std::isfinite(length) || length<=1.0e-5F) return std::nullopt;
    normal={normal.x/length,normal.y/length,normal.z/length};
    if (normal.z<0.0F) normal={-normal.x,-normal.y,-normal.z};
    return Plane{normal,-(normal.x*first.x+normal.y*first.y+normal.z*first.z)};
}

std::optional<PlaneEstimate> estimate_ground_plane(const std::vector<models::PointXYZIRT>& samples,const GroundRemovalConfig& config) {
    if (samples.size()<3) return std::nullopt;
    std::mt19937 generator(0x56495354U);
    std::uniform_int_distribution<std::size_t> pick(0,samples.size()-1);
    const auto minimum_up_dot=std::cos(degrees_to_radians(config.normal_tolerance_deg));
    std::size_t best_inlier_count=0; std::optional<Plane> best;
    for (std::size_t iteration=0; iteration<ransac_iterations; ++iteration) {
        const auto first=pick(generator),second=pick(generator),third=pick(generator);
        if (first==second || first==third || second==third) continue;
        auto candidate=plane_from_points(samples[first],samples[second],samples[third]);
        if (!candidate || candidate->normal.z<minimum_up_dot) continue;
        if (config.mode==GroundMode::hybrid) {
            const auto prior_error=std::fabs(candidate->normal.x*config.mount_x_m + candidate->normal.y*config.mount_y_m + candidate->normal.z*config.floor_z_m + candidate->d);
            if (prior_error>std::max(0.25F,3.0F*config.distance_threshold_m)) continue;
        }
        const auto count=static_cast<std::size_t>(std::count_if(samples.begin(),samples.end(),[&](const auto& point){return point_plane_distance(*candidate,point)<=config.distance_threshold_m;}));
        if (count>best_inlier_count) { best_inlier_count=count; best=*candidate; }
    }
    const auto ratio=static_cast<float>(best_inlier_count)/static_cast<float>(samples.size());
    if (!best || ratio<config.minimum_inlier_ratio) return std::nullopt;
    std::vector<float> offsets; offsets.reserve(best_inlier_count);
    for (const auto& point:samples) if (point_plane_distance(*best,point)<=config.distance_threshold_m)
        offsets.push_back(-(best->normal.x*point.x+best->normal.y*point.y+best->normal.z*point.z));
    const auto middle=offsets.begin()+static_cast<std::ptrdiff_t>(offsets.size()/2);
    std::nth_element(offsets.begin(),middle,offsets.end()); best->d=*middle;
    std::vector<float> residuals;
    residuals.reserve(best_inlier_count);
    double sum=0.0, square_sum=0.0;
    for (const auto& point:samples) {
        const auto residual=point_plane_distance(*best,point);
        if (residual<=config.distance_threshold_m) {
            residuals.push_back(residual); sum+=residual; square_sum+=static_cast<double>(residual)*residual;
        }
    }
    std::sort(residuals.begin(),residuals.end());
    const auto count=residuals.size();
    const auto p95_index=count==0 ? 0 : std::min(count-1,static_cast<std::size_t>(std::ceil(0.95*static_cast<double>(count)))-1);
    return PlaneEstimate{
        *best,
        count,
        static_cast<float>(count)/static_cast<float>(samples.size()),
        count==0 ? 0.0F : static_cast<float>(sum/static_cast<double>(count)),
        count==0 ? 0.0F : static_cast<float>(std::sqrt(square_sum/static_cast<double>(count))),
        count==0 ? 0.0F : residuals[p95_index],
    };
}

std::string current_exception_message() {
    try { throw; } catch (const std::exception& error) { return error.what(); } catch (...) { return "unknown preprocessing failure"; }
}

}  // namespace

const char* to_string(GroundMode mode) noexcept {
    switch (mode) { case GroundMode::static_height:return "static"; case GroundMode::ransac:return "ransac"; case GroundMode::hybrid:return "hybrid"; }
    return "unknown";
}

GroundMode parse_ground_mode(const std::string& value) {
    const auto normalized=lower_copy(value);
    if (normalized=="static" || normalized=="fixed" || normalized=="fixed-height") return GroundMode::static_height;
    if (normalized=="ransac") return GroundMode::ransac;
    if (normalized=="hybrid") return GroundMode::hybrid;
    throw std::invalid_argument("unsupported GroundMode '"+value+"'; expected hybrid, ransac, or static");
}

class PointCloudPreprocessor::Impl {
public:
    explicit Impl(PreprocessingConfig value):config(std::move(value)) {
        validate(config);
        if (config.ground_removal && config.ground_removal->mode==GroundMode::static_height) calibration_finished=true;
    }

    void update_imu(const models::ImuFrame& sample) {
        if (!config.ground_removal || !config.ground_removal->use_imu) return;
        const Vec3 acceleration{sample.linear_acceleration_x_m_s2,sample.linear_acceleration_y_m_s2,sample.linear_acceleration_z_m_s2};
        const Vec3 angular{sample.angular_velocity_x_rad_s,sample.angular_velocity_y_rad_s,sample.angular_velocity_z_rad_s};
        const auto acceleration_magnitude=norm(acceleration), angular_speed=norm(angular);
        if (!std::isfinite(acceleration_magnitude) || acceleration_magnitude<7.0F || acceleration_magnitude>12.5F || !std::isfinite(angular_speed) || angular_speed>0.5F) return;
        const auto measured_up=normalized(acceleration);
        if (!imu_up) {
            imu_up=measured_up;
            if (!calibration_finished) { calibration_samples.clear(); calibration_frame_count=0; }
        } else {
            constexpr float weight=0.10F;
            imu_up=normalized({imu_up->x*(1.0F-weight)+measured_up.x*weight,imu_up->y*(1.0F-weight)+measured_up.y*weight,imu_up->z*(1.0F-weight)+measured_up.z*weight});
        }
    }

    models::PointCloudFrame process(models::PointCloudFrame frame) {
        const auto input_point_count=frame.points.size();
        const auto min2=config.min_distance_m*config.min_distance_m,max2=config.max_distance_m*config.max_distance_m;
        frame.points.erase(std::remove_if(frame.points.begin(),frame.points.end(),[=](const auto& point){
            if (!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) return true;
            const auto distance2=point.x*point.x+point.y*point.y+point.z*point.z;
            return distance2<min2 || distance2>max2;
        }),frame.points.end());
        if (config.ground_removal) transform_to_world(frame,*config.ground_removal);
        if (config.region_of_interest) {
            const auto roi=*config.region_of_interest;
            frame.points.erase(std::remove_if(frame.points.begin(),frame.points.end(),[=](const auto& point){return !contains(roi,point);}),frame.points.end());
        }
        if (config.voxel_size_m) {
            const auto voxel=*config.voxel_size_m; std::unordered_set<VoxelKey,VoxelKeyHash> occupied; occupied.reserve(frame.points.size());
            frame.points.erase(std::remove_if(frame.points.begin(),frame.points.end(),[&](const auto& point){
                const VoxelKey key{static_cast<int>(std::floor(point.x/voxel)),static_cast<int>(std::floor(point.y/voxel)),static_cast<int>(std::floor(point.z/voxel))};
                return !occupied.insert(key).second;
            }),frame.points.end());
        }
        if (config.ground_removal) {
            calibrate_if_needed(frame.points,*config.ground_removal);
            const auto removed=remove_ground(frame,*config.ground_removal);
            update_ground_status(frame.timestamp_ns,input_point_count,removed,frame.points.size(),*config.ground_removal);
        }
        return frame;
    }

    bool ground_calibrated() const noexcept {
        if (!config.ground_removal) return false;
        return config.ground_removal->mode==GroundMode::static_height || calibrated_plane.has_value();
    }
    bool using_imu_orientation() const noexcept { return imu_up.has_value(); }
    models::GroundStatus ground_status() const { return latest_ground_status; }

private:
    Matrix3 current_rotation(const GroundRemovalConfig& ground) const {
        if (ground.use_imu && imu_up) return imu_rotation(*imu_up,ground.mount_yaw_deg);
        return euler_rotation(ground.mount_roll_deg,ground.mount_pitch_deg,ground.mount_yaw_deg);
    }
    void transform_to_world(models::PointCloudFrame& frame,const GroundRemovalConfig& ground) const {
        const auto rotation=current_rotation(ground);
        for (auto& point:frame.points) {
            const auto world=multiply(rotation,Vec3{point.x,point.y,point.z});
            point.x=world.x+ground.mount_x_m; point.y=world.y+ground.mount_y_m; point.z=world.z+ground.mount_z_m;
        }
    }
    void calibrate_if_needed(const std::vector<models::PointXYZIRT>& points,const GroundRemovalConfig& ground) {
        if (ground.mode==GroundMode::static_height || calibration_finished) return;
        ++calibration_frame_count;
        if (!points.empty() && calibration_samples.size()<maximum_calibration_samples) {
            const auto available=maximum_calibration_samples-calibration_samples.size();
            const auto wanted=std::min(maximum_samples_per_frame,available);
            const auto stride=std::max<std::size_t>(1,points.size()/wanted);
            for (std::size_t index=0;index<points.size() && calibration_samples.size()<maximum_calibration_samples;index+=stride) calibration_samples.push_back(points[index]);
        }
        if (calibration_frame_count<ground.calibration_frames) return;
        calibration_sample_count=calibration_samples.size();
        const auto estimate=estimate_ground_plane(calibration_samples,ground);
        if (estimate) {
            calibrated_plane=estimate->plane;
            ground_inlier_count=estimate->inlier_count;
            ground_inlier_ratio=estimate->inlier_ratio;
            mean_residual_m=estimate->mean_residual_m;
            rms_residual_m=estimate->rms_residual_m;
            p95_residual_m=estimate->p95_residual_m;
        }
        calibration_finished=true; calibration_samples.clear(); calibration_samples.shrink_to_fit();
    }
    std::optional<Plane> selected_ground_plane(const GroundRemovalConfig& ground) const {
        std::optional<Plane> plane;
        if (ground.mode==GroundMode::static_height || ground.mode==GroundMode::hybrid || calibration_finished) plane=calibrated_plane.value_or(static_ground_plane(ground));
        return plane;
    }
    std::size_t remove_ground(models::PointCloudFrame& frame,const GroundRemovalConfig& ground) const {
        const auto plane=selected_ground_plane(ground);
        if (!plane) return 0;
        const auto before=frame.points.size();
        frame.points.erase(std::remove_if(frame.points.begin(),frame.points.end(),[&](const auto& point){return point_plane_distance(*plane,point)<=ground.distance_threshold_m;}),frame.points.end());
        return before-frame.points.size();
    }
    void update_ground_status(std::uint64_t timestamp,std::size_t input_count,std::size_t removed,std::size_t output_count,const GroundRemovalConfig& ground) {
        models::GroundStatus status;
        status.timestamp_ns=timestamp;
        status.configured_mode=to_string(ground.mode);
        status.calibrated=ground.mode==GroundMode::static_height || calibrated_plane.has_value();
        status.using_imu=ground.use_imu && imu_up.has_value();
        status.using_static_fallback=!calibrated_plane.has_value() &&
            (ground.mode==GroundMode::hybrid || calibration_finished);
        if (ground.mode==GroundMode::static_height || calibrated_plane) status.state=models::GroundState::valid;
        else if (!calibration_finished) status.state=models::GroundState::calibrating;
        else status.state=models::GroundState::static_fallback;
        if (const auto plane=selected_ground_plane(ground)) {
            status.plane_a=plane->normal.x; status.plane_b=plane->normal.y; status.plane_c=plane->normal.z; status.plane_d=plane->d;
            status.ground_tilt_deg=std::acos(std::clamp(plane->normal.z,-1.0F,1.0F))*180.0F/pi;
            status.sensor_to_ground_distance_m=std::fabs(
                plane->normal.x*ground.mount_x_m+plane->normal.y*ground.mount_y_m+plane->normal.z*ground.mount_z_m+plane->d);
        }
        status.expected_ground_distance_m=std::fabs(ground.mount_z_m-ground.floor_z_m);
        status.ground_height_error_m=status.sensor_to_ground_distance_m-status.expected_ground_distance_m;
        status.calibration_frame_count=calibration_frame_count;
        status.calibration_sample_count=calibration_finished ? calibration_sample_count : calibration_samples.size();
        status.ground_inlier_count=ground_inlier_count;
        status.ground_inlier_ratio=ground_inlier_ratio;
        status.mean_residual_m=mean_residual_m; status.rms_residual_m=rms_residual_m; status.p95_residual_m=p95_residual_m;
        status.input_point_count=input_count; status.removed_ground_point_count=removed; status.output_point_count=output_count;
        const auto ground_candidate_count=removed+output_count;
        status.removed_ground_ratio=ground_candidate_count==0 ? 0.0F :
            static_cast<float>(removed)/static_cast<float>(ground_candidate_count);
        latest_ground_status=std::move(status);
    }
    PreprocessingConfig config;
    std::optional<Vec3> imu_up;
    std::vector<models::PointXYZIRT> calibration_samples;
    std::size_t calibration_frame_count{};
    bool calibration_finished{};
    std::optional<Plane> calibrated_plane;
    std::size_t calibration_sample_count{};
    std::size_t ground_inlier_count{};
    float ground_inlier_ratio{};
    float mean_residual_m{};
    float rms_residual_m{};
    float p95_residual_m{};
    models::GroundStatus latest_ground_status;
};

PointCloudPreprocessor::PointCloudPreprocessor(PreprocessingConfig config):impl_(std::make_unique<Impl>(std::move(config))) {}
PointCloudPreprocessor::~PointCloudPreprocessor()=default;
PointCloudPreprocessor::PointCloudPreprocessor(PointCloudPreprocessor&&) noexcept=default;
PointCloudPreprocessor& PointCloudPreprocessor::operator=(PointCloudPreprocessor&&) noexcept=default;
void PointCloudPreprocessor::update_imu(const models::ImuFrame& sample) { impl_->update_imu(sample); }
models::PointCloudFrame PointCloudPreprocessor::process(models::PointCloudFrame frame) { return impl_->process(std::move(frame)); }
bool PointCloudPreprocessor::ground_calibrated() const noexcept { return impl_->ground_calibrated(); }
bool PointCloudPreprocessor::using_imu_orientation() const noexcept { return impl_->using_imu_orientation(); }
models::GroundStatus PointCloudPreprocessor::ground_status() const { return impl_->ground_status(); }

models::PointCloudFrame preprocess_point_cloud(models::PointCloudFrame frame,const PreprocessingConfig& config) {
    PointCloudPreprocessor processor(config); return processor.process(std::move(frame));
}

platform::WorkerHandle spawn_preprocessing_worker(platform::MessageBus& bus,platform::ThreadConfig thread_config,platform::StopToken stop,PreprocessingConfig config,PreprocessingCompletion on_complete) {
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto pointcloud_input=inputs.subscribe<devices::LidarPointCloudMessage>(bus,models::topics::pointcloud_decoded);
    auto imu_input=inputs.subscribe<devices::LidarImuMessage>(bus,models::topics::lidar_imu);
    auto publisher=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_processed);
    auto ground_publisher=bus.publisher<models::LidarGroundStatusMessage>(models::topics::ground_status);
    return platform::spawn_worker(std::move(thread_config),[stop,inputs=std::move(inputs),pointcloud_input=std::move(pointcloud_input),imu_input=std::move(imu_input),publisher=std::move(publisher),ground_publisher=std::move(ground_publisher),config=std::move(config),on_complete=std::move(on_complete)]() mutable {
        try {
            PreprocessingReport report; PointCloudPreprocessor processor(std::move(config)); bool pointcloud_closed=false;
            std::optional<models::GroundState> last_ground_state;
            while (!pointcloud_closed) {
                const auto ready=inputs.wait();
                if (imu_input.is_ready(ready)) {
                    std::shared_ptr<const devices::LidarImuMessage> message;
                    while (imu_input.try_receive(message)==platform::ReceiveStatus::message) processor.update_imu(message->payload);
                }
                if (pointcloud_input.is_ready(ready)) {
                    std::shared_ptr<const devices::LidarPointCloudMessage> message;
                    while (true) {
                        const auto status=pointcloud_input.try_receive(message);
                        if (status==platform::ReceiveStatus::closed) { pointcloud_closed=true; break; }
                        if (status!=platform::ReceiveStatus::message) break;
                        auto processed=processor.process(message->payload); ++report.message_count; report.point_count+=processed.points.size();
                        auto ground_status=processor.ground_status();
                        if (!ground_status.configured_mode.empty() && (!last_ground_state || *last_ground_state!=ground_status.state)) {
                            std::cout << "Ground status: " << models::to_string(ground_status.state)
                                      << ", mode=" << ground_status.configured_mode
                                      << ", tilt=" << ground_status.ground_tilt_deg << " deg"
                                      << ", height error=" << ground_status.ground_height_error_m << " m"
                                      << ", inliers=" << ground_status.ground_inlier_ratio*100.0F << "%"
                                      << ", RMS residual=" << ground_status.rms_residual_m << " m\n";
                            last_ground_state=ground_status.state;
                        }
                        if (!ground_status.configured_mode.empty()) {
                            ground_publisher.publish(models::LidarGroundStatusMessage(
                                message->lidar_id,message->sequence,message->sensor_timestamp_ns,
                                message->received_timestamp_ns,std::move(ground_status)));
                        }
                        publisher.publish(devices::LidarPointCloudMessage(message->lidar_id,message->sequence,message->sensor_timestamp_ns,message->received_timestamp_ns,std::move(processed)));
                    }
                }
            }
            report.dropped_message_count=pointcloud_input.dropped_messages(); on_complete(report,{});
        } catch (...) { stop.request_stop(); on_complete(std::nullopt,current_exception_message()); }
    });
}

}  // namespace vista::application
