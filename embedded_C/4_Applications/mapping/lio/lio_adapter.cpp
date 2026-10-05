#include "4_Applications/mapping/lio/lio_adapter.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vista::application {
void validate_lio_config(const LioConfig& c) {
    for (const auto* values : {&c.initial_position_m, &c.initial_rpy_deg,
                              &c.lidar_to_imu_translation_m, &c.lidar_to_imu_rpy_deg})
        for (double v : *values) if (!std::isfinite(v)) throw std::invalid_argument("non-finite LIO pose/extrinsic");
    for (double v : {c.scan_voxel_m, c.map_voxel_m, c.local_radius_m,
                    c.maximum_imu_gap_s, c.minimum_match_ratio, c.maximum_residual_m,
                    c.maximum_translation_step_m, c.maximum_rotation_step_deg})
        if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("LIO limits must be positive and finite");
    if (c.minimum_match_ratio > 1 || c.maximum_rotation_step_deg > 180 ||
        c.local_max_points < 100 || c.scan_max_points < 100 ||
        c.local_max_points > 2'000'000 || c.scan_max_points > 100'000 ||
        c.initialization_samples < 20 || c.initialization_samples > 100'000 ||
        c.synchronization_timeout.count() <= 0 || c.synchronization_timeout.count() > 60'000)
        throw std::invalid_argument("invalid LIO budget, timing, or quality limit");
}
LioSynchronizer::LioSynchronizer(double maximum_gap_s):maximum_gap_(maximum_gap_s) {}
bool LioSynchronizer::add(TimedImu sample) {
    if (!std::isfinite(sample.time_s)) return false;
    for (auto values : {sample.acceleration, sample.angular_velocity})
        for (double v : values) if (!std::isfinite(v)) return false;
    double acc2=0,gyro2=0;
    for(std::size_t i=0;i<3;++i){acc2+=sample.acceleration[i]*sample.acceleration[i];gyro2+=sample.angular_velocity[i]*sample.angular_velocity[i];}
    if(acc2>200.0*200.0 || gyro2>35.0*35.0) return false;
    if (!samples_.empty() && sample.time_s <= samples_.back().time_s) return false;
    samples_.push_back(sample);
    while (samples_.size() > 4096) samples_.pop_front();
    return true;
}
std::optional<std::vector<TimedImu>> LioSynchronizer::take(double begin_s, double end_s) {
    if (!std::isfinite(begin_s) || !std::isfinite(end_s) || end_s < begin_s ||
        samples_.size() < 2 || samples_.front().time_s > begin_s || samples_.back().time_s < end_s)
        return std::nullopt;
    auto first = samples_.begin();
    while (first + 1 != samples_.end() && (first + 1)->time_s <= begin_s) ++first;
    auto last = first;
    while (last != samples_.end() && last->time_s < end_s) ++last;
    if (last == samples_.end()) return std::nullopt;
    for (auto it = first + 1; it <= last; ++it)
        if (it->time_s - (it - 1)->time_s > maximum_gap_) return std::nullopt;
    std::vector<TimedImu> batch(first, last + 1);
    // Unitree scan boundaries can overlap. Preserve a bounded lookback plus
    // its left bracket rather than discarding everything through scan end.
    while (samples_.size() > 2 && samples_[1].time_s < end_s-0.5) samples_.pop_front();
    return batch;
}
void LioSynchronizer::clear() { samples_.clear(); }
} // namespace vista::application
