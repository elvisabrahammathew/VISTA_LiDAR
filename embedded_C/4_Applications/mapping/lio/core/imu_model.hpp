#pragma once
#include "4_Applications/mapping/lio/lio.hpp"

namespace vista::application::lio_detail {
// Point-LIO state_output: pos, rot, extrinsic R/T, velocity, angular velocity,
// specific acceleration, gravity, gyro bias, acceleration bias (30 DOF).
// Matches Estimator.cpp::process_noise_cov_output. The upstream filter applies
// Q * dt^2; values are covariance diagonals, not standard deviations.
inline std::array<double,30> imu_process_noise_diagonal(const LioConfig& c) {
    std::array<double,30> q{};
    for(std::size_t i=0;i<3;++i) {
        q[12+i]=c.velocity_process_covariance;
        q[15+i]=c.angular_velocity_process_covariance;
        q[18+i]=c.acceleration_process_covariance;
        q[24+i]=c.imu_gyroscope_bias_covariance;
        q[27+i]=c.imu_acceleration_bias_covariance;
    }
    return q;
}
// laserMapping.cpp::P_init_output; extrinsic estimation remains disabled.
inline std::array<double,30> imu_initial_covariance_diagonal() {
    std::array<double,30> p{};p.fill(0.01);
    for(std::size_t i=6;i<12;++i)p[i]=0.0001;
    for(std::size_t i=21;i<24;++i)p[i]=0.0001;
    for(std::size_t i=24;i<30;++i)p[i]=0.001;
    return p;
}
} // namespace vista::application::lio_detail
