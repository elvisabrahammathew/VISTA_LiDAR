#pragma once
#include <deque>
#include <optional>
#include "4_Applications/mapping/lio/lio.hpp"

namespace vista::application {
// Bounded IMU buffer; returns a batch only with both temporal brackets and no gaps.
class LioSynchronizer {
public:
    explicit LioSynchronizer(double maximum_gap_s);
    bool add(TimedImu sample);
    std::optional<std::vector<TimedImu>> take(double begin_s, double end_s);
    void clear();
    std::size_t size() const noexcept { return samples_.size(); }
private:
    double maximum_gap_;
    std::deque<TimedImu> samples_;
};
} // namespace vista::application
