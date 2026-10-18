#pragma once
#include <array>
#include <cstring>
#include <deque>
#include "models/imu/imu_message.hpp"

namespace vista::application {
// Identity is sensor timestamp AND all decoded sample bits, never arrival time
// or just a repeated stationary value. Only previously admitted samples qualify.
class ImuSampleHistory {
public:
    bool contains_duplicate(const models::ImuFrame& frame) const {
        if(!frame.timestamp_ns || samples_.empty() || frame.timestamp_ns>samples_.back().timestamp_ns ||
           samples_.back().timestamp_ns-frame.timestamp_ns>window_ns)return false;
        const auto value=bits(frame);
        for(const auto& entry:samples_)
            if(entry.timestamp_ns==frame.timestamp_ns && entry.value==value)return true;
        return false;
    }
    void remember(const models::ImuFrame& frame) {
        if(!frame.timestamp_ns)return; // Timestamp-less external IMU is never deduplicated by value.
        if(!samples_.empty() && frame.timestamp_ns<=samples_.back().timestamp_ns)return;
        samples_.push_back({frame.timestamp_ns,bits(frame)});
        while(samples_.size()>512 || frame.timestamp_ns-samples_.front().timestamp_ns>window_ns)
            samples_.pop_front();
    }
    void clear(){samples_.clear();}
    std::size_t size() const{return samples_.size();}
private:
    static constexpr std::uint64_t window_ns=100'000'000;
    using Bits=std::array<std::uint32_t,10>;
    static Bits bits(const models::ImuFrame& f) {
        const std::array<float,10> values{f.orientation_x,f.orientation_y,f.orientation_z,f.orientation_w,
            f.angular_velocity_x_rad_s,f.angular_velocity_y_rad_s,f.angular_velocity_z_rad_s,
            f.linear_acceleration_x_m_s2,f.linear_acceleration_y_m_s2,f.linear_acceleration_z_m_s2};
        static_assert(sizeof(float)==sizeof(std::uint32_t),"IMU requires 32-bit float");
        Bits result{};std::memcpy(result.data(),values.data(),sizeof(values));return result;
    }
    struct Entry{std::uint64_t timestamp_ns;Bits value;};
    std::deque<Entry> samples_;
};
} // namespace vista::application
