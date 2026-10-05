#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace vista::platform {
// A process-wide monotonic time domain, unrelated to calendar time/NTP changes.
inline std::uint64_t monotonic_timestamp_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// One instance per hardware clock (Unitree LiDAR and its IMU share one).
// Anchor at RAW reception, preserve sensor deltas even when USB batches samples.
// This estimates arrival latency; it is NOT hardware time synchronization.
class MeasurementClock {
public:
    std::optional<std::uint64_t> align(std::optional<std::uint64_t> sensor_ns,
                                       std::uint64_t received_monotonic_ns) {
        if (failed_ || received_monotonic_ns == 0 ||
            (previous_host_ && received_monotonic_ns < *previous_host_)) {
            failed_ = true;
            return std::nullopt;
        }
        previous_host_ = received_monotonic_ns;
        if (!sensor_ns) return received_monotonic_ns;
        if (!latest_sensor_) {
            latest_sensor_ = *sensor_ns;
            offset_ns_ = static_cast<long double>(received_monotonic_ns) - *sensor_ns;
        } else if (*sensor_ns < *latest_sensor_ && *latest_sensor_ - *sensor_ns > 500'000'000ULL) {
            // Slight interleaving of IMU/scan timestamps is legitimate. A reset
            // beyond the supported scan duration must not reset the world frame.
            failed_ = true;
            return std::nullopt;
        } else if (*sensor_ns > *latest_sensor_) {
            const auto delta = *sensor_ns - *latest_sensor_;
            const long double observed = static_cast<long double>(received_monotonic_ns) - *sensor_ns;
            // Gradually follow drift, limiting time-rate distortion to 100 ppm.
            // Late arrivals affect the anchor much less than low-latency arrivals.
            const auto correction = (observed < offset_ns_ ? 0.05L : 0.0001L) * (observed - offset_ns_);
            const auto limit = static_cast<long double>(delta) * 0.0001L;
            offset_ns_ += std::clamp(correction, -limit, limit);
            latest_sensor_ = *sensor_ns;
        }
        const long double result = static_cast<long double>(*sensor_ns) + offset_ns_;
        if (result < 0 || result > std::numeric_limits<std::uint64_t>::max()) {
            failed_ = true;
            return std::nullopt;
        }
        return static_cast<std::uint64_t>(result);
    }
private:
    std::optional<std::uint64_t> latest_sensor_, previous_host_;
    long double offset_ns_{};
    bool failed_{};
};

// Signed relative timestamps also support a scan assembled from earlier packets.
inline std::optional<std::uint64_t> shift_measurement_time(
    std::optional<std::uint64_t> host_measurement_ns,
    std::uint64_t sensor_ns, std::uint64_t sensor_reference_ns) {
    if (!host_measurement_ns) return std::nullopt;
    const long double result = static_cast<long double>(*host_measurement_ns) +
        static_cast<long double>(sensor_ns) - sensor_reference_ns;
    if (result < 0 || result > std::numeric_limits<std::uint64_t>::max()) return std::nullopt;
    return static_cast<std::uint64_t>(result);
}
} // namespace vista::platform
