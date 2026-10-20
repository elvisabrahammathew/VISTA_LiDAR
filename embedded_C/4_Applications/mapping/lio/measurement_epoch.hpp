#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vista::application {

// Admission fence for independent topic queues. Integrated LiDAR/IMU announce
// the SAME boundary; external IMUs may announce their own boundary. Either
// invalidates all old temporal state, without requiring equal device counters.
class MeasurementEpochGate {
public:
    enum class Result { accept, changed, stale, invalid };
    Result observe(std::size_t stream, std::uint64_t epoch,
                   std::uint64_t started, std::uint64_t received) {
        if (stream >= stamps_.size() || received == 0 || started > received ||
            ((epoch == 0) != (started == 0))) return Result::invalid;
        if (received < cutoff_) return Result::stale;
        auto& stamp = stamps_[stream];
        if (epoch < stamp.epoch) return Result::stale;
        if ((epoch == stamp.epoch && started != stamp.started) ||
            (epoch > stamp.epoch && started <= stamp.started)) return Result::invalid;
        stamp = {epoch, started};
        if (started > cutoff_) {
            cutoff_ = started;
            return Result::changed;
        }
        return Result::accept;
    }
private:
    struct Stamp { std::uint64_t epoch{}, started{}; };
    std::array<Stamp, 2> stamps_{};
    std::uint64_t cutoff_{};
};
} // namespace vista::application
