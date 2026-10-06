#pragma once
#include <deque>
#include <optional>
#include "4_Applications/mapping/lio/lio.hpp"

namespace vista::application {
// Real elapsed host time, independent of scan rate or sensor clock epochs.
class LioRecoveryTimer {
public:
    bool update(models::LocalizationState state, std::chrono::steady_clock::time_point now) {
        if(state!=models::LocalizationState::lost){lost_since_.reset();return false;}
        if(!lost_since_)lost_since_=now;
        if(now-*lost_since_<std::chrono::seconds(5))return false;
        lost_since_.reset();return true;
    }
private:
    std::optional<std::chrono::steady_clock::time_point> lost_since_;
};
// Bounded IMU buffer; returns a batch only with both temporal brackets and no gaps.
class LioSynchronizer {
public:
    explicit LioSynchronizer(double maximum_gap_s);
    bool add(TimedImu sample);
    std::optional<std::vector<TimedImu>> take(double begin_s, double end_s);
    // Call only after the engine reports its integrated horizon. Keep overlap
    // history and its left bracket; taking a batch alone must not consume it.
    void retain_from(double integrated_time_s);
    // Diagnostic only: never consumes samples or relaxes the coverage gate.
    std::string describe_coverage(double begin_s, double end_s) const;
    void clear();
    std::size_t size() const noexcept { return samples_.size(); }
    std::uint64_t evicted_samples() const noexcept { return evicted_samples_; }
private:
    double maximum_gap_;
    std::deque<TimedImu> samples_;
    std::uint64_t evicted_samples_{};
};
} // namespace vista::application
