// ROS-free Point-LIO adapter, upstream Unitree 18ed5976d8fab2bd8a5148c26a40692bd3c0dc91.
// The actual 30-DOF state, dynamics and two-measurement EKF are generated from
// upstream Estimator.{h,cpp}/IKFoM. Derived code: upstream GPL-2.0; see README.
#include "4_Applications/mapping/lio/lio.hpp"
#include "4_Applications/mapping/lio/core/imu_model.hpp"
#include "4_Applications/mapping/lio/core/portable_types.hpp"
#include <point_lio_models.hpp>
#include <ikd_Tree.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <unordered_set>

namespace vista::application {
namespace {
using Vec=Eigen::Vector3d;
using Mat=Eigen::Matrix3d;
using State=point_lio_native::state_output;
using Filter=esekfom::esekf<State,30,point_lio_native::input_ikfom>;
using Point=lio_port::PointXYZINormal;
using Tree=KD_TREE<Point>;
using Points=Tree::PointVector;
constexpr double pi=3.14159265358979323846;
Vec vector(const std::array<double,3>& a){return {a[0],a[1],a[2]};}
Vec xyz(const Point& p){return {p.x,p.y,p.z};}
Point point(const Vec& v){Point p;p.x=static_cast<float>(v.x());p.y=static_cast<float>(v.y());p.z=static_cast<float>(v.z());return p;}
Mat rotation(const std::array<double,3>& rpy){return (Eigen::AngleAxisd(rpy[2]*pi/180,Vec::UnitZ())*Eigen::AngleAxisd(rpy[1]*pi/180,Vec::UnitY())*Eigen::AngleAxisd(rpy[0]*pi/180,Vec::UnitX())).toRotationMatrix();}
Mat skew(const Vec& v){Mat m;m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m;}
Mat exp_rotation(const Vec& v){const double n=v.norm();return n<1e-12?Mat::Identity():Eigen::AngleAxisd(n,v/n).toRotationMatrix();}
double angle_deg(const State& a,const State& b){return Eigen::AngleAxisd(a.rot.normalized().toRotationMatrix().transpose()*b.rot.normalized().toRotationMatrix()).angle()*180/pi;}
struct Cell {std::int64_t x,y,z;bool operator==(const Cell& b) const{return x==b.x && y==b.y && z==b.z;}};
struct Hash {std::size_t operator()(const Cell& c) const{return std::hash<std::int64_t>{}(c.x)^(std::hash<std::int64_t>{}(c.y)<<1)^(std::hash<std::int64_t>{}(c.z)<<2);}};
Cell cell(const Vec& p,double v){return {static_cast<std::int64_t>(std::floor(p.x()/v)),static_cast<std::int64_t>(std::floor(p.y()/v)),static_cast<std::int64_t>(std::floor(p.z()/v))};}
struct Knot {double t;Vec p,v,a,w;Mat r;};
struct ImuWindow {
    std::deque<TimedImu> samples;Vec acceleration{Vec::Zero()},gyro{Vec::Zero()};double acceleration_rms{},gyro_rms{},duration{};
    void add(const LioScan& scan,std::size_t required){
        for(const auto& s:scan.imu){
            if(s.time_s>scan.end_s || (!samples.empty() && s.time_s<=samples.back().time_s))continue;
            samples.push_back(s);
            while(samples.size()>required && samples.front().time_s<s.time_s-.75)samples.pop_front();
            while(samples.size()>100'000)samples.pop_front();
        }
        acceleration.setZero();gyro.setZero();acceleration_rms=gyro_rms=duration=0;
        if(samples.empty())return;
        for(const auto& s:samples){acceleration+=vector(s.acceleration);gyro+=vector(s.angular_velocity);}
        acceleration/=static_cast<double>(samples.size());gyro/=static_cast<double>(samples.size());
        for(const auto& s:samples){acceleration_rms+=(vector(s.acceleration)-acceleration).squaredNorm();gyro_rms+=(vector(s.angular_velocity)-gyro).squaredNorm();}
        acceleration_rms=std::sqrt(acceleration_rms/samples.size());gyro_rms=std::sqrt(gyro_rms/samples.size());duration=samples.back().time_s-samples.front().time_s;
    }
    bool stable(double scale) const{return duration>=.5-1e-8 && acceleration.norm()*scale>=7 && acceleration.norm()*scale<=12.5 && gyro.norm()<.15 && acceleration_rms*scale<.35 && gyro_rms<.025;}
};
}

class LioEngine::Impl {
public:
    explicit Impl(LioConfig c):config(std::move(c)){
        validate_lio_config(config);
        filter.init_dyn_share_modified_2h(point_lio_native::get_f_output,point_lio_native::df_dx_output,laser_measurement,imu_measurement);
        auto s=filter.get_x();s.offset_R_L_I=rotation(config.lidar_to_imu_rpy_deg);s.offset_T_L_I=vector(config.lidar_to_imu_translation_m);
        s.rot=rotation(config.initial_rpy_deg)*s.offset_R_L_I.normalized().toRotationMatrix().transpose();
        s.pos=vector(config.initial_position_m)-s.rot.normalized()*s.offset_T_L_I;
        filter.change_x(s);initial_anchor=s;configure_covariances();
    }
    std::optional<double> propagation_time_s() const{return initialized?std::optional<double>(filter_time):std::nullopt;}
    LioEngineStatistics statistics() const{return counts;}
    models::LocalizationStatus restart_initialization(){
        auto anchor=last_accepted.value_or(initial_anchor);anchor.vel.setZero();anchor.omg.setZero();anchor.acc.setZero();anchor.bg.setZero();anchor.ba.setZero();
        filter.change_x(anchor);configure_covariances();initialized=fatal=false;fatal_reason.clear();first_rejection.reset();imu_window=ImuWindow{};
        filter_time=cov_time=last_imu_update=0;history.clear();recovery_streak=0;recovering=last_accepted.has_value();
        if(!last_accepted){tree.reset();local_points.clear();map_cells.clear();}
        return diagnostic(models::LocalizationState::initializing,"automatic IMU reinitialization; keep stationary; retained world map requires Point-LIO match");
    }
    LioOutput process(const LioScan& scan){
        current_time=scan.end_s;
        if(fatal)return {diagnostic(models::LocalizationState::lost,fatal_reason+"; automatic initialization after 5s LOST"),std::nullopt};
        if(scan.imu.size()<2 || !std::isfinite(scan.begin_s) || !std::isfinite(scan.end_s) || scan.end_s<scan.begin_s)return reject("IMU does not bracket scan");
        for(std::size_t i=0;i<scan.imu.size();++i){
            const auto& m=scan.imu[i];
            if(!std::isfinite(m.time_s) || !vector(m.acceleration).allFinite() || !vector(m.angular_velocity).allFinite() || vector(m.acceleration).norm()>10'000 || vector(m.angular_velocity).norm()>10'000)return reject("non-finite IMU sample or corrupt magnitude");
            if(i && (m.time_s<=scan.imu[i-1].time_s || m.time_s-scan.imu[i-1].time_s>config.maximum_imu_gap_s))return reject("IMU gap/out-of-order");
        }
        if(scan.imu.front().time_s>scan.begin_s+1e-8 || scan.imu.back().time_s<scan.end_s-1e-8)return reject("incomplete temporal coverage");
        imu_window.add(scan,config.initialization_samples);
        if(!initialized){
            if(imu_window.samples.size()>=config.initialization_samples && imu_window.stable(9.81/config.imu_acceleration_norm)){
                auto s=filter.get_x();s.bg=imu_window.gyro;s.omg.setZero();s.vel.setZero();s.ba.setZero();
                // Keep Mount as initial pose; initialize gravity in that world
                // frame. Normalize measurements by explicit acc_norm, as upstream.
                s.gravity=-(s.rot.normalized()*imu_window.acceleration.normalized())*9.81;
                s.acc=-(s.rot.conjugate().normalized()*s.gravity);
                filter.change_x(s);configure_covariances();initialized=true;filter_time=cov_time=last_imu_update=scan.end_s;remember();
            }
            std::ostringstream r;r.imbue(std::locale::classic());r<<std::fixed<<std::setprecision(3)
                <<(initialized?"IMU initialized; accumulating Point-LIO local map":"IMU initialization; keep stationary")
                <<"; samples="<<imu_window.samples.size()<<"/"<<config.initialization_samples<<"; duration_s="<<imu_window.duration
                <<"/0.5; accel_norm="<<imu_window.acceleration.norm()<<"; accel_rms="<<imu_window.acceleration_rms<<"/0.35; gyro_norm="<<imu_window.gyro.norm()<<"/0.15; gyro_rms="<<imu_window.gyro_rms<<"/0.025";
            return {diagnostic(models::LocalizationState::initializing,r.str()),std::nullopt};
        }
        if(scan.end_s<=filter_time || scan.imu.front().time_s>filter_time+1e-8 || scan.end_s-filter_time>2){
            return reject(scan.end_s<=filter_time?"scan end is not monotonic":scan.imu.front().time_s>filter_time+1e-8?"IMU batch misses integrated horizon":"propagation backlog exceeds 2s",true);
        }
        struct ActiveGuard {Impl*& slot;Impl* previous;ActiveGuard(Impl*& p,Impl* now):slot(p),previous(p){slot=now;}~ActiveGuard(){slot=previous;}} guard(active,this);
        std::vector<std::size_t> order;order.reserve(scan.cloud.points.size());
        std::vector<double> times(scan.cloud.points.size());
        for(std::size_t i=0;i<scan.cloud.points.size();++i){
            const auto& p=scan.cloud.points[i];const Vec v(p.x,p.y,p.z);
            const double t=scan.begin_s+static_cast<double>((static_cast<long double>(p.timestamp_ns)-scan.cloud.timestamp_ns)*1e-9L);
            if(!v.allFinite() || v.norm()<.1 || v.norm()>200 || t<scan.begin_s-1e-6 || t>scan.end_s+1e-6)continue;
            times[i]=t;order.push_back(i);
        }
        std::stable_sort(order.begin(),order.end(),[&](auto a,auto b){return times[a]<times[b];});
        std::unordered_set<Cell,Hash> seen;std::vector<std::size_t> choices;
        for(auto i:order){const auto& p=scan.cloud.points[i];const Vec v(p.x,p.y,p.z);if(v.norm()>=config.minimum_range_m && seen.insert(cell(v,config.scan_voxel_m)).second)choices.push_back(i);}
        std::vector<bool> selected(scan.cloud.points.size(),false);
        if(choices.size()>config.scan_max_points){
            for(std::size_t i=0;i<config.scan_max_points;++i)selected[choices[i*choices.size()/config.scan_max_points]]=true;
        }else for(auto i:choices)selected[i]=true;
        if(choices.size()<100)return reject("insufficient scan geometry");
        const State saved_state=filter.get_x();auto saved_cov=filter.get_P();const auto saved_history=history;
        const double saved_time=filter_time,saved_cov_time=cov_time,saved_imu_time=last_imu_update;
        models::PointCloudFrame world=scan.cloud;world.points.clear();world.points.reserve(order.size());
        Points additions;matches=examined=0;residual_sum=0;normal_information.setZero();pose_information.setZero();
        std::size_t imu_cursor=0;while(imu_cursor<scan.imu.size() && scan.imu[imu_cursor].time_s<=last_imu_update+1e-10)++imu_cursor;
        for(std::size_t begin=0;begin<order.size();){
            std::size_t end=begin+1;const double t=times[order[begin]];
            while(end<order.size() && times[order[end]]==t)++end;
            if(t>=filter_time-1e-10){
                advance(std::max(t,filter_time),scan.imu,imu_cursor);
                if(tree)for(std::size_t j=begin;j<end;++j)if(selected[order[j]]){
                    const auto& p=scan.cloud.points[order[j]];measurement_point=Vec(p.x,p.y,p.z);++examined;
                    // Upstream scalar/identical-time point measurement EKF.
                    if(filter.update_iterated_dyn_share_modified())++counts.lidar_updates;
                }
                remember();
            }
            const auto pose=pose_at(t);
            if(pose)for(std::size_t j=begin;j<end;++j){
                const auto index=order[j];auto p=scan.cloud.points[index];
                const Vec body=vector_transform(Vec(p.x,p.y,p.z));const Vec v=pose->r*body+pose->p;
                const Vec origin=pose->r*filter.get_x().offset_T_L_I+pose->p;
                p.x=static_cast<float>(v.x());p.y=static_cast<float>(v.y());p.z=static_cast<float>(v.z());
                p.ray_origin_world_m=std::array<float,3>{static_cast<float>(origin.x()),static_cast<float>(origin.y()),static_cast<float>(origin.z())};
                world.points.push_back(p);
                if(selected[index]){auto q=point(v);q.intensity=p.intensity;additions.push_back(q);}
            }
            begin=end;
        }
        advance(scan.end_s,scan.imu,imu_cursor);remember();prune_history();
        if(!finite_state()){
            auto s=saved_state;filter.change_x(s);filter.change_P(saved_cov);history=saved_history;filter_time=saved_time;cov_time=saved_cov_time;last_imu_update=saved_imu_time;
            return reject("non-finite Point-LIO state",true);
        }
        if(!tree){
            refresh_map(additions,true);
            return {diagnostic(models::LocalizationState::initializing,"Point-LIO local map seeded; waiting for LiDAR pose correction"),std::nullopt};
        }
        const double ratio=examined?static_cast<double>(matches)/examined:0;
        residual=matches?residual_sum/matches:0;
        Eigen::SelfAdjointEigenSolver<Mat> normals(normal_information);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> poses(pose_information);
        const double normal_min=normals.info()==Eigen::Success?normals.eigenvalues().minCoeff():-1;
        const double pose_min=poses.info()==Eigen::Success?poses.eigenvalues().minCoeff():-1;
        const auto corrected=filter.get_x();const auto reference=last_accepted.value_or(saved_state);
        std::string failure;
        if(matches<30)failure="insufficient LiDAR correspondences";
        else if(normal_min<=1e-3 || pose_min<=1e-4)failure="degenerate LiDAR geometry";
        else if(ratio<config.minimum_match_ratio)failure="LiDAR match ratio below limit";
        else if(residual>config.maximum_residual_m)failure="LiDAR residual above limit";
        else if(filter.get_P().block<3,3>(0,0).trace()>10)failure="position covariance above limit";
        else if((corrected.pos-reference.pos).norm()>config.maximum_translation_step_m || angle_deg(reference,corrected)>config.maximum_rotation_step_deg)failure="pose jump rejected";
        if(!failure.empty()){
            // Discard all tentative LiDAR corrections from this scan. Continue
            // only IMU-as-output propagation; an invalid pose never reaches map.
            auto s=saved_state;filter.change_x(s);filter.change_P(saved_cov);history=saved_history;filter_time=saved_time;cov_time=saved_cov_time;last_imu_update=saved_imu_time;
            std::size_t cursor=0;while(cursor<scan.imu.size() && scan.imu[cursor].time_s<=last_imu_update+1e-10)++cursor;
            advance(scan.end_s,scan.imu,cursor);remember();prune_history();
            std::ostringstream r;r.imbue(std::locale::classic());r<<failure<<"; matches="<<matches<<"/"<<examined<<"; ratio="<<ratio<<"; residual_m="<<residual<<"; normal_eigen_min="<<normal_min<<"; pose_eigen_min="<<pose_min;
            return reject(r.str());
        }
        // A rejected established pose needs several consistent scans against an
        // unchanged trusted map, not one passing frame that reseeds the world.
        if(recovering && ++recovery_streak<3)return {diagnostic(models::LocalizationState::degraded,"Point-LIO recovery confirmation "+std::to_string(recovery_streak)+"/3; map held"),std::nullopt};
        recovering=false;recovery_streak=0;first_rejection.reset();last_accepted=corrected;refresh_map(additions,false);
        const Vec origin=corrected.pos+corrected.rot.normalized()*corrected.offset_T_L_I;
        world.sensor_origin_world_m=std::array<float,3>{static_cast<float>(origin.x()),static_cast<float>(origin.y()),static_cast<float>(origin.z())};
        auto status=diagnostic(models::LocalizationState::tracking,"Point-LIO LiDAR-corrected pose; IMU-as-output; point-time updates");status.pose_valid=true;
        return {std::move(status),std::move(world)};
    }
private:
    void configure_covariances(){
        noise.setZero();auto q=lio_detail::imu_process_noise_diagonal(config);
        for(int i=0;i<30;++i)noise(i,i)=q[static_cast<std::size_t>(i)];
        auto p=Filter::cov::Zero().eval();auto values=lio_detail::imu_initial_covariance_diagonal();
        for(int i=0;i<30;++i)p(i,i)=values[static_cast<std::size_t>(i)];filter.change_P(p);
    }
    Vec vector_transform(const Vec& p) const{const auto& s=filter.get_x();return s.offset_R_L_I.normalized()*p+s.offset_T_L_I;}
    void predict_to(double time){
        double dt=time-filter_time;if(dt<=1e-12)return;
        filter.predict(dt,noise,input,true,false);filter_time=time;
    }
    void advance(double time,const std::vector<TimedImu>& imu,std::size_t& cursor){
        while(cursor<imu.size() && imu[cursor].time_s<=time+1e-10){
            const auto& m=imu[cursor++];if(m.time_s<=last_imu_update+1e-10)continue;
            predict_to(m.time_s);double dt=m.time_s-cov_time;
            if(dt>0){filter.predict(dt,noise,input,false,true);cov_time=m.time_s;}
            latest_imu=m;filter.update_iterated_dyn_share_IMU();++counts.imu_updates;last_imu_update=m.time_s;remember();
        }
        predict_to(time);
    }
    void remember(){const auto& s=filter.get_x();const Mat r=s.rot.normalized().toRotationMatrix();Knot k{filter_time,s.pos,s.vel,r*s.acc+s.gravity,s.omg,r};
        if(!history.empty() && std::abs(history.back().t-filter_time)<1e-10)history.back()=k;else history.push_back(k);
    }
    void prune_history(){while(history.size()>1 && (history[1].t<filter_time-.5 || history.size()>100'000))history.pop_front();}
    std::optional<Knot> pose_at(double t) const{
        if(history.empty() || t<history.front().t-1e-8)return std::nullopt;
        auto it=std::upper_bound(history.begin(),history.end(),t+1e-10,[](double v,const Knot& k){return v<k.t;});
        if(it==history.begin())return std::nullopt;Knot k=*(it-1);const double dt=std::max(0.0,t-k.t);
        k.p+=k.v*dt+.5*k.a*dt*dt;k.r=k.r*exp_rotation(k.w*dt);k.t=t;return k;
    }
    bool finite_state() const{const auto& s=filter.get_x();return s.pos.allFinite() && s.vel.allFinite() && s.acc.allFinite() && s.omg.allFinite() && s.gravity.allFinite() && s.bg.allFinite() && s.ba.allFinite() && s.rot.coeffs().allFinite() && filter.get_P().allFinite();}
    static void imu_measurement(State& s,esekfom::dyn_share_modified<double>& d){
        if(!active){d.valid=false;return;}auto& a=*active;
        // Estimator.cpp::h_model_IMU_output, including per-axis saturation mask.
        d.z_IMU.head<3>()=vector(a.latest_imu.angular_velocity)-s.omg-s.bg;
        d.z_IMU.tail<3>()=vector(a.latest_imu.acceleration)*9.81/a.config.imu_acceleration_norm-s.acc-s.ba;
        for(int i=0;i<6;++i){const bool gyro=i<3;const auto axis=static_cast<std::size_t>(i%3);
            d.R_IMU[i]=gyro?a.config.imu_gyroscope_covariance:a.config.imu_acceleration_covariance;
            const double value=gyro?a.latest_imu.angular_velocity[axis]:a.latest_imu.acceleration[axis];
            const double limit=gyro?a.config.imu_gyroscope_saturation:a.config.imu_acceleration_saturation;
            d.satu_check[i]=std::abs(value)>=.99*limit;if(d.satu_check[i]){d.z_IMU[i]=0;++a.counts.saturated_imu_axes;}
        }
    }
    static void laser_measurement(State& s,esekfom::dyn_share_modified<double>& d){
        if(!active){d.valid=false;return;}active->match(s,d);
    }
    void match(State& s,esekfom::dyn_share_modified<double>& d){
        const Vec body=s.offset_R_L_I.normalized()*measurement_point+s.offset_T_L_I;
        const Vec world=s.rot.normalized()*body+s.pos;Points near;std::vector<float> distances;
        tree->Nearest_Search(point(world),5,near,distances,2.236);
        if(near.size()!=5 || distances.back()>5){d.valid=false;return;}
        Vec center=Vec::Zero();for(const auto& p:near)center+=xyz(p);center/=5;
        Mat scatter=Mat::Zero();for(const auto& p:near){const Vec v=xyz(p)-center;scatter+=v*v.transpose();}
        Eigen::SelfAdjointEigenSolver<Mat> eig(scatter);
        if(eig.info()!=Eigen::Success || eig.eigenvalues()[1]<1e-5 || eig.eigenvalues()[0]>.10*eig.eigenvalues()[1]){d.valid=false;return;}
        const Vec normal=eig.eigenvectors().col(0);
        for(const auto& p:near)if(std::abs(normal.dot(xyz(p)-center))>config.plane_threshold_m){d.valid=false;return;}
        const double error=normal.dot(world-center);
        if(measurement_point.norm()<=81*error*error){d.valid=false;return;}
        const Vec a=skew(body)*(s.rot.conjugate().normalized()*normal);
        d.M_Noise=config.lidar_measurement_covariance;d.h_x=Eigen::MatrixXd::Zero(1,12);d.h_x.block<1,3>(0,0)=normal.transpose();d.h_x.block<1,3>(0,3)=a.transpose();
        d.z.resize(1);d.z[0]=-error;++matches;residual_sum+=std::abs(error);normal_information+=normal*normal.transpose();
        const Eigen::Matrix<double,1,6> row=d.h_x.leftCols<6>();pose_information+=row.transpose()*row;
    }
    void refresh_map(const Points& points,bool bootstrap){
        const auto& s=filter.get_x();const Vec origin=s.pos+s.rot.normalized()*s.offset_T_L_I;Points additions;
        for(const auto& p:points){const Vec v=xyz(p);const auto key=cell(v,config.map_voxel_m);
            if(!v.allFinite() || (v-origin).norm()>config.local_radius_m || map_cells.count(key))continue;
            if(tree){Points nearest;std::vector<float> distance;tree->Nearest_Search(p,1,nearest,distance);if(!distance.empty() && distance[0]<.25*config.map_voxel_m*config.map_voxel_m)continue;}
            map_cells.insert(key);additions.push_back(p);
        }
        local_points.insert(local_points.end(),additions.begin(),additions.end());
        if(bootstrap || local_points.size()>config.local_max_points || (origin-last_map_center).norm()>config.local_radius_m*.2){
            local_points.erase(std::remove_if(local_points.begin(),local_points.end(),[&](const Point& p){return (xyz(p)-origin).norm()>config.local_radius_m;}),local_points.end());
            if(local_points.size()>config.local_max_points){std::nth_element(local_points.begin(),local_points.begin()+static_cast<std::ptrdiff_t>(config.local_max_points),local_points.end(),[&](const Point& a,const Point& b){return (xyz(a)-origin).squaredNorm()<(xyz(b)-origin).squaredNorm();});local_points.resize(config.local_max_points);}
            map_cells.clear();for(const auto& p:local_points)map_cells.insert(cell(xyz(p),config.map_voxel_m));
            tree.reset();if(!local_points.empty()){tree=std::make_unique<Tree>();tree->set_downsample_param(static_cast<float>(config.map_voxel_m));tree->Build(local_points);}last_map_center=origin;
        }else if(!additions.empty())tree->Add_Points(additions,false);
    }
    models::LocalizationStatus diagnostic(models::LocalizationState state,std::string reason) const{
        models::LocalizationStatus d;d.state=state;d.reason=std::move(reason);d.local_map_points=local_points.size();d.rejected_scans=rejected;
        const auto s=last_accepted.value_or(initial_anchor);const Vec origin=s.pos+s.rot.normalized()*s.offset_T_L_I;const Eigen::Quaterniond q(s.rot.normalized().toRotationMatrix()*s.offset_R_L_I.normalized().toRotationMatrix());
        d.position_m={origin.x(),origin.y(),origin.z()};d.orientation_xyzw={q.x(),q.y(),q.z(),q.w()};d.match_ratio=examined?static_cast<double>(matches)/examined:0;d.residual_m=residual;d.position_variance=filter.get_P().block<3,3>(0,0).trace();return d;
    }
    LioOutput reject(std::string reason,bool lost=false){
        ++rejected;recovery_streak=0;recovering=last_accepted.has_value();
        if(initialized && std::isfinite(current_time)){if(!first_rejection)first_rejection=current_time;if(current_time-*first_rejection>=2)fatal=true;}
        if(lost || fatal){fatal=true;fatal_reason=reason;}
        return {diagnostic(fatal?models::LocalizationState::lost:models::LocalizationState::degraded,std::move(reason)),std::nullopt};
    }
    LioConfig config;Filter filter;Filter::processnoisecovariance noise;point_lio_native::input_ikfom input;
    State initial_anchor;std::optional<State> last_accepted;ImuWindow imu_window;TimedImu latest_imu;
    std::unique_ptr<Tree> tree;Points local_points;std::unordered_set<Cell,Hash> map_cells;Vec last_map_center{Vec::Zero()},measurement_point{Vec::Zero()};
    std::deque<Knot> history;LioEngineStatistics counts;bool initialized{},fatal{},recovering{};unsigned recovery_streak{};
    double filter_time{},cov_time{},last_imu_update{},current_time{},residual{},residual_sum{};
    std::size_t matches{},examined{};std::uint64_t rejected{};std::optional<double> first_rejection;std::string fatal_reason;
    Mat normal_information{Mat::Zero()};Eigen::Matrix<double,6,6> pose_information{Eigen::Matrix<double,6,6>::Zero()};
    static thread_local Impl* active;
};
thread_local LioEngine::Impl* LioEngine::Impl::active=nullptr;
LioEngine::LioEngine(LioConfig config):impl_(std::make_unique<Impl>(std::move(config))){}
LioEngine::~LioEngine()=default;
LioOutput LioEngine::process(const LioScan& scan){return impl_->process(scan);}
std::optional<double> LioEngine::propagation_time_s() const{return impl_->propagation_time_s();}
models::LocalizationStatus LioEngine::restart_initialization(){return impl_->restart_initialization();}
LioEngineStatistics LioEngine::statistics() const{return impl_->statistics();}
} // namespace vista::application
