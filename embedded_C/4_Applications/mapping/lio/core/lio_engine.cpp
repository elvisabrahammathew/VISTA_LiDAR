// ROS-free adaptation of FAST-LIO's IMU propagation/deskew and
// laserMapping.cpp::h_share_model, pinned upstream 7cc4175de6f8.
// FAST-LIO LICENSE (GPLv2) applies to derived algorithm code; see README.md.
#include "4_Applications/mapping/lio/lio.hpp"
#include "4_Applications/mapping/lio/core/portable_types.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <use-ikfom.hpp>
#include <ikd_Tree.h>

namespace vista::application {
namespace {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Point = lio_port::PointXYZINormal;
using Tree = KD_TREE<Point>;
using Points = Tree::PointVector;
using Filter = esekfom::esekf<state_ikfom,12,input_ikfom>;
Mat rotation(const std::array<double,3>& rpy) {
    constexpr double rad=3.14159265358979323846/180.0;
    return (Eigen::AngleAxisd(rpy[2]*rad,Vec::UnitZ())*
            Eigen::AngleAxisd(rpy[1]*rad,Vec::UnitY())*
            Eigen::AngleAxisd(rpy[0]*rad,Vec::UnitX())).toRotationMatrix();
}
Vec vector(const std::array<double,3>& a) { return {a[0],a[1],a[2]}; }
Vec xyz(const Point& p) { return {p.x,p.y,p.z}; }
Point point(const Vec& v) { Point p; p.x=static_cast<float>(v.x()); p.y=static_cast<float>(v.y()); p.z=static_cast<float>(v.z()); return p; }
Mat skew(const Vec& v) { Mat m; m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return m; }
Mat exp_rotation(const Vec& v) {
    const double angle=v.norm();
    return angle<1e-12 ? Mat::Identity() : Eigen::AngleAxisd(angle,v/angle).toRotationMatrix();
}
struct Cell {
    std::int64_t x,y,z;
    bool operator==(const Cell& b) const {return x==b.x && y==b.y && z==b.z;}
};
struct Hash {
    std::size_t operator()(const Cell& c) const {
        return std::hash<std::int64_t>{}(c.x) ^ (std::hash<std::int64_t>{}(c.y)<<1) ^ (std::hash<std::int64_t>{}(c.z)<<2);
    }
};
Cell cell(const Vec& p,double v) { return {static_cast<std::int64_t>(std::floor(p.x()/v)),static_cast<std::int64_t>(std::floor(p.y()/v)),static_cast<std::int64_t>(std::floor(p.z()/v))}; }
TimedImu interpolate(const std::vector<TimedImu>& samples,double t) {
    auto tail=std::lower_bound(samples.begin(),samples.end(),t,[](const auto& s,double time){return s.time_s<time;});
    if(tail==samples.begin()) return *tail;
    if(tail==samples.end()) return samples.back();
    const auto& head=*(tail-1);
    const double alpha=(t-head.time_s)/(tail->time_s-head.time_s);
    TimedImu out;out.time_s=t;
    for(std::size_t i=0;i<3;++i){out.acceleration[i]=head.acceleration[i]+alpha*(tail->acceleration[i]-head.acceleration[i]);out.angular_velocity[i]=head.angular_velocity[i]+alpha*(tail->angular_velocity[i]-head.angular_velocity[i]);}
    return out;
}
struct Knot { double t; Vec p,v,a,w; Mat r; };
}

class LioEngine::Impl {
public:
    explicit Impl(LioConfig c):config(std::move(c)) {
        validate_lio_config(config);
        double limits[23];std::fill(std::begin(limits),std::end(limits),0.001);
        filter.init_dyn_share(get_f,df_dx,df_dw,measurement,4,limits);
        auto s=filter.get_x();
        s.offset_R_L_I=rotation(config.lidar_to_imu_rpy_deg);
        s.offset_T_L_I=vector(config.lidar_to_imu_translation_m);
        // Mount specifies the INITIAL LiDAR pose. Convert it to the IMU pose.
        const Mat imu_r=rotation(config.initial_rpy_deg)*s.offset_R_L_I.toRotationMatrix().transpose();
        s.rot=imu_r;
        s.pos=vector(config.initial_position_m)-imu_r*s.offset_T_L_I;
        filter.change_x(s);
        noise=process_noise_cov();
    }
    LioOutput process(const LioScan& scan) {
        LioOutput out;
        if(fatal){out.status=diagnostic(models::LocalizationState::lost,fatal_reason+"; restart required");return out;}
        if(scan.imu.size()<2 || scan.end_s<scan.begin_s || !std::isfinite(scan.end_s))
            return reject("IMU does not bracket scan");
        for(std::size_t i=1;i<scan.imu.size();++i)
            if(scan.imu[i].time_s<=scan.imu[i-1].time_s || scan.imu[i].time_s-scan.imu[i-1].time_s>config.maximum_imu_gap_s)
                return reject("IMU gap/out-of-order");
        for(const auto& imu:scan.imu)
            if(!std::isfinite(imu.time_s) || !vector(imu.acceleration).allFinite() || !vector(imu.angular_velocity).allFinite() ||
               vector(imu.acceleration).norm()>200 || vector(imu.angular_velocity).norm()>35)
                return reject("non-finite IMU sample");
        if(scan.imu.front().time_s>scan.begin_s+1e-8 || scan.imu.back().time_s<scan.end_s-1e-8)
            return reject("incomplete temporal coverage");
        if(!initialized) {
            for(const auto& sample:scan.imu) {
                if(sample.time_s<=last_init_time || sample.time_s>scan.end_s) continue;
                last_init_time=sample.time_s;
                const Vec acc=vector(sample.acceleration),gyro=vector(sample.angular_velocity);
                if(acc.norm()<7.0 || acc.norm()>12.5 || gyro.norm()>0.15 ||
                   (init_count>5 && (acc-mean_acc).norm()>0.5)) {
                    init_count=0;mean_acc.setZero();mean_gyro.setZero();continue;
                }
                if(init_count==0) initialization_begin=sample.time_s;
                ++init_count;mean_acc+=(acc-mean_acc)/static_cast<double>(init_count);mean_gyro+=(gyro-mean_gyro)/static_cast<double>(init_count);
            }
            if(init_count>=config.initialization_samples && last_init_time-initialization_begin>=0.5-1e-8) {
                auto s=filter.get_x();
                s.bg=mean_gyro;
                // Gravity is in world coordinates; preserve the configured initial pose.
                const Vec gravity=-(s.rot.toRotationMatrix()*mean_acc.normalized())*9.81;
                s.grav=S2(gravity);
                filter.change_x(s);
                Filter::cov p=Filter::cov::Identity()*0.01;
                p.block<6,6>(6,6)=Eigen::Matrix<double,6,6>::Identity()*1e-8;
                filter.change_P(p);
                initialized=true;filter_time=scan.end_s;
            }
            out.status=diagnostic(models::LocalizationState::initializing,"keep sensor stationary during IMU initialization");return out;
        }
        if(scan.end_s<=filter_time || scan.imu.front().time_s>filter_time+1e-8 ||
           scan.end_s-filter_time>2.0) {
            fatal=true;return reject("scan clock reset or uncovered interval",true);
        }
        // Integrate exactly to scan end with interpolated measurements; no extrapolation.
        std::vector<double> steps{filter_time};
        for(const auto& sample:scan.imu) if(sample.time_s>filter_time && sample.time_s<scan.end_s) steps.push_back(sample.time_s);
        steps.push_back(scan.end_s);
        auto& knots=pose_history;
        for(std::size_t i=1;i<steps.size();++i) {
            const double start=steps[i-1];double dt=steps[i]-start;
            if(dt<=0 || dt>config.maximum_imu_gap_s) return reject("prediction interval exceeds IMU limit");
            const auto measured=interpolate(scan.imu,start+0.5*dt);
            input_ikfom in;in.acc=vector(measured.acceleration);in.gyro=vector(measured.angular_velocity);
            const auto s=filter.get_x();
            const Vec w=in.gyro-s.bg,a=s.rot.toRotationMatrix()*(in.acc-s.ba)+s.grav.vec;
            knots.push_back({start,s.pos,s.vel,a,w,s.rot.toRotationMatrix()});
            filter.predict(dt,noise,in);
        }
        filter_time=scan.end_s;
        // Match the synchronizer's overlap budget, retaining one left bracket.
        while(knots.size()>1 && knots[1].t<scan.end_s-0.5) knots.erase(knots.begin());
        auto predicted=filter.get_x();
        auto prediction_cov=filter.get_P();
        models::PointCloudFrame deskewed=scan.cloud;
        deskewed.points.clear();deskewed.points.reserve(scan.cloud.points.size());
        for(auto p:scan.cloud.points) {
            const Vec raw(p.x,p.y,p.z);
            if(!raw.allFinite() || raw.norm()<0.1 || raw.norm()>200) continue;
            double t=scan.begin_s+static_cast<double>(static_cast<long double>(p.timestamp_ns)-scan.cloud.timestamp_ns)*1e-9;
            if(t<scan.begin_s-1e-6 || t>scan.end_s+1e-6) continue;
            if(t<knots.front().t && knots.front().t-t<1e-8) t=knots.front().t;
            auto it=std::upper_bound(knots.begin(),knots.end(),t,[](double time,const Knot& k){return time<k.t;});
            if(it==knots.begin()) return reject("point time precedes propagated pose");
            const auto& k=*(it-1);const double dt=t-k.t;
            const Vec sensor_at_t=predicted.offset_R_L_I*raw+predicted.offset_T_L_I;
            const Vec world=(k.r*exp_rotation(k.w*dt))*sensor_at_t+k.p+k.v*dt+0.5*k.a*dt*dt;
            const Vec end=predicted.offset_R_L_I.conjugate()*(predicted.rot.conjugate()*(world-predicted.pos)-predicted.offset_T_L_I);
            const Vec ray_origin=(k.r*exp_rotation(k.w*dt))*predicted.offset_T_L_I+k.p+k.v*dt+0.5*k.a*dt*dt;
            p.ray_origin_world_m=std::array<float,3>{static_cast<float>(ray_origin.x()),static_cast<float>(ray_origin.y()),static_cast<float>(ray_origin.z())};
            p.x=static_cast<float>(end.x());p.y=static_cast<float>(end.y());p.z=static_cast<float>(end.z());deskewed.points.push_back(p);
        }
        scan_points.clear();std::unordered_set<Cell,Hash> seen;
        // Spread the budget over the scan rather than taking only its first sector.
        const auto stride=std::max<std::size_t>(1,deskewed.points.size()/config.scan_max_points);
        for(std::size_t i=0;i<deskewed.points.size() && scan_points.size()<config.scan_max_points;i+=stride) {
            const auto& p=deskewed.points[i];const Vec v(p.x,p.y,p.z);
            if(seen.insert(cell(v,config.scan_voxel_m)).second){auto q=point(v);q.intensity=p.intensity;scan_points.push_back(q);}
        }
        if(scan_points.size()<100) return reject("insufficient scan geometry");
        if(!tree) {
            refresh_map(predicted,true);
            out.status=diagnostic(models::LocalizationState::initializing,"local map seeded; waiting for LiDAR pose correction");return out;
        }
        active=this;matches=0;residual=0;normal_information.setZero();pose_information.setZero();
        double solve_time=0;
        filter.update_iterated_dyn_share_modified(0.001,solve_time);
        active=nullptr;
        const auto corrected=filter.get_x();
        const auto covariance=filter.get_P();
        const double ratio=static_cast<double>(matches)/static_cast<double>(scan_points.size());
        Eigen::SelfAdjointEigenSolver<Mat> observability(normal_information);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> pose_observability(pose_information);
        const bool geometry_ok=matches>=30 && observability.info()==Eigen::Success && observability.eigenvalues().minCoeff()>1e-3 &&
            pose_observability.info()==Eigen::Success && pose_observability.eigenvalues().minCoeff()>1e-4;
        const double translation=(corrected.pos-predicted.pos).norm();
        const double angle=Eigen::AngleAxisd(predicted.rot.toRotationMatrix().transpose()*corrected.rot.toRotationMatrix()).angle()*180.0/3.141592653589793;
        if(!geometry_ok || ratio<config.minimum_match_ratio || residual>config.maximum_residual_m ||
           !corrected.pos.allFinite() || !covariance.allFinite() ||
           covariance.block<3,3>(0,0).trace()>10 || translation>config.maximum_translation_step_m || angle>config.maximum_rotation_step_deg) {
            filter.change_x(predicted);filter.change_P(prediction_cov);
            return reject("weak/degenerate LiDAR match or pose quality limit");
        }
        if(last_accepted && ((corrected.pos-last_accepted->pos).norm()>config.maximum_translation_step_m ||
           Eigen::AngleAxisd(last_accepted->rot.toRotationMatrix().transpose()*corrected.rot.toRotationMatrix()).angle()*180.0/3.141592653589793>config.maximum_rotation_step_deg)) {
            filter.change_x(predicted);filter.change_P(prediction_cov);return reject("pose jump rejected");
        }
        consecutive_rejections=0;last_accepted=corrected;
        // Bring retained propagation knots into the accepted corrected frame.
        // This is the same rigid correction applied to per-point ray origins.
        const Mat correction=corrected.rot.toRotationMatrix()*predicted.rot.toRotationMatrix().transpose();
        for(auto& k:knots) {
            k.p=correction*(k.p-predicted.pos)+corrected.pos;
            k.v=correction*k.v;k.a=correction*k.a;k.r=correction*k.r;
        }
        refresh_map(corrected,false);
        const Vec origin=corrected.pos+corrected.rot.toRotationMatrix()*corrected.offset_T_L_I;
        deskewed.sensor_origin_world_m=std::array<float,3>{static_cast<float>(origin.x()),static_cast<float>(origin.y()),static_cast<float>(origin.z())};
        for(auto& p:deskewed.points){
            const Vec v=corrected.rot.toRotationMatrix()*(corrected.offset_R_L_I.toRotationMatrix()*Vec(p.x,p.y,p.z)+corrected.offset_T_L_I)+corrected.pos;
            p.x=static_cast<float>(v.x());p.y=static_cast<float>(v.y());p.z=static_cast<float>(v.z());
            if(p.ray_origin_world_m){
                const auto& o=*p.ray_origin_world_m;
                const Vec corrected_origin=corrected.rot.toRotationMatrix()*predicted.rot.toRotationMatrix().transpose()*(Vec(o[0],o[1],o[2])-predicted.pos)+corrected.pos;
                p.ray_origin_world_m=std::array<float,3>{static_cast<float>(corrected_origin.x()),static_cast<float>(corrected_origin.y()),static_cast<float>(corrected_origin.z())};
            }
        }
        out.world_cloud=std::move(deskewed);
        out.status=diagnostic(models::LocalizationState::tracking,"LiDAR-corrected pose");out.status.pose_valid=true;
        return out;
    }
private:
    static void measurement(state_ikfom& state,esekfom::dyn_share_datastruct<double>& data) {
        if(!active){data.valid=false;return;}active->match(state,data);
    }
    void match(state_ikfom& s,esekfom::dyn_share_datastruct<double>& data) {
        std::vector<Eigen::Matrix<double,1,12>> rows;std::vector<double> errors;
        normal_information.setZero();pose_information.setZero();residual=0;
        for(const auto& p:scan_points) {
            const Vec sensor=xyz(p),body=s.offset_R_L_I*sensor+s.offset_T_L_I,world=s.rot*body+s.pos;
            Points near;std::vector<float> distances;tree->Nearest_Search(point(world),5,near,distances);
            if(near.size()<5 || distances.back()>2.25F) continue;
            Vec center=Vec::Zero();for(const auto& n:near) center+=xyz(n);center/=5;
            Mat scatter=Mat::Zero();for(const auto& n:near){const Vec d=xyz(n)-center;scatter+=d*d.transpose();}
            Eigen::SelfAdjointEigenSolver<Mat> eig(scatter);
            if(eig.info()!=Eigen::Success || eig.eigenvalues()[1]<1e-5 || eig.eigenvalues()[0]>0.02*eig.eigenvalues()[1]) continue;
            const Vec normal=eig.eigenvectors().col(0);
            bool plane_ok=true;for(const auto& n:near) if(std::abs(normal.dot(xyz(n)-center))>0.10) plane_ok=false;
            const double error=normal.dot(world-center);
            if(!plane_ok || std::abs(error)>std::min(0.3,0.1*std::sqrt(std::max(0.1,sensor.norm())))) continue;
            const Vec c=s.rot.conjugate()*normal,a=skew(body)*c;
            Eigen::Matrix<double,1,12> row;row.setZero();row.block<1,3>(0,0)=normal.transpose();row.block<1,3>(0,3)=a.transpose();
            rows.push_back(row);errors.push_back(-error);residual+=std::abs(error);normal_information+=normal*normal.transpose();
            pose_information+=row.leftCols<6>().transpose()*row.leftCols<6>();
        }
        matches=rows.size();
        if(matches<30){data.valid=false;return;}
        residual/=static_cast<double>(matches);
        data.h_x.resize(static_cast<Eigen::Index>(matches),12);data.h.resize(static_cast<Eigen::Index>(matches));
        for(std::size_t i=0;i<matches;++i){data.h_x.row(static_cast<Eigen::Index>(i))=rows[i];data.h[static_cast<Eigen::Index>(i)]=errors[i];}
    }
    void refresh_map(const state_ikfom& s,bool bootstrap) {
        const Vec origin=s.pos+s.rot.toRotationMatrix()*s.offset_T_L_I;
        Points additions;
        for(const auto& p:scan_points) {
            const Vec v=s.rot.toRotationMatrix()*(s.offset_R_L_I.toRotationMatrix()*xyz(p)+s.offset_T_L_I)+s.pos;
            const auto key=cell(v,config.map_voxel_m);
            if((v-origin).norm()>config.local_radius_m || map_cells.count(key)!=0) continue;
            if(tree) {
                // Tiny pose corrections can put almost-identical surface points
                // on opposite voxel boundaries. Do not let those duplicates
                // crowd out the five spatial neighbors used for plane fitting.
                Points nearest;std::vector<float> distances;
                tree->Nearest_Search(point(v),1,nearest,distances);
                if(!distances.empty() && distances.front()<0.25*config.map_voxel_m*config.map_voxel_m) continue;
            }
            map_cells.insert(key);additions.push_back(point(v));
        }
        local_points.insert(local_points.end(),additions.begin(),additions.end());
        const bool rebuild=bootstrap || local_points.size()>config.local_max_points ||
            (origin-last_map_center).norm()>config.local_radius_m*0.20;
        if(rebuild) {
            local_points.erase(std::remove_if(local_points.begin(),local_points.end(),[&](const Point& p){return (xyz(p)-origin).norm()>config.local_radius_m;}),local_points.end());
            if(local_points.size()>config.local_max_points){std::nth_element(local_points.begin(),local_points.begin()+static_cast<std::ptrdiff_t>(config.local_max_points),local_points.end(),[&](const Point& a,const Point& b){return (xyz(a)-origin).squaredNorm()<(xyz(b)-origin).squaredNorm();});local_points.resize(config.local_max_points);}
            map_cells.clear();for(const auto& p:local_points)map_cells.insert(cell(xyz(p),config.map_voxel_m));
            // Destroy before Build: upstream Build alone retains its old sentinel.
            tree.reset();tree=std::make_unique<Tree>();tree->set_downsample_param(static_cast<float>(config.map_voxel_m));tree->Build(local_points);last_map_center=origin;
        } else if(!additions.empty()) {
            tree->Add_Points(additions,false); // Incremental balanced-tree insertion.
        }
    }
    models::LocalizationStatus diagnostic(models::LocalizationState state,std::string reason) const {
        models::LocalizationStatus d;d.state=state;d.reason=std::move(reason);d.local_map_points=local_points.size();d.rejected_scans=rejected;
        const auto s=filter.get_x();const Vec origin=s.pos+s.rot.toRotationMatrix()*s.offset_T_L_I;const Eigen::Quaterniond q(s.rot.toRotationMatrix()*s.offset_R_L_I.toRotationMatrix());
        d.position_m={origin.x(),origin.y(),origin.z()};d.orientation_xyzw={q.x(),q.y(),q.z(),q.w()};
        d.match_ratio=scan_points.empty()?0:static_cast<double>(matches)/static_cast<double>(scan_points.size());d.residual_m=residual;d.position_variance=filter.get_P().block<3,3>(0,0).trace();return d;
    }
    LioOutput reject(std::string reason,bool lost=false) {
        ++rejected;++consecutive_rejections;
        if(initialized && consecutive_rejections>=30) fatal=true;
        if(lost || fatal){fatal=true;fatal_reason=reason;}
        return {diagnostic(lost||fatal?models::LocalizationState::lost:models::LocalizationState::degraded,std::move(reason)),std::nullopt};
    }
    LioConfig config;Filter filter;Filter::processnoisecovariance noise;
    std::unique_ptr<Tree> tree;Points local_points,scan_points;std::unordered_set<Cell,Hash> map_cells;Vec last_map_center{Vec::Zero()};
    bool initialized{},fatal{};double filter_time{},last_init_time{-std::numeric_limits<double>::infinity()};
    std::vector<Knot> pose_history;
    std::string fatal_reason;
    std::size_t init_count{},matches{};double initialization_begin{};Vec mean_acc{Vec::Zero()},mean_gyro{Vec::Zero()};Mat normal_information{Mat::Zero()};
    Eigen::Matrix<double,6,6> pose_information{Eigen::Matrix<double,6,6>::Zero()};
    double residual{};std::uint64_t rejected{},consecutive_rejections{};std::optional<state_ikfom> last_accepted;
    static thread_local Impl* active;
};
thread_local LioEngine::Impl* LioEngine::Impl::active=nullptr;
LioEngine::LioEngine(LioConfig config):impl_(std::make_unique<Impl>(std::move(config))) {}
LioEngine::~LioEngine()=default;
LioOutput LioEngine::process(const LioScan& scan){return impl_->process(scan);}
} // namespace vista::application
