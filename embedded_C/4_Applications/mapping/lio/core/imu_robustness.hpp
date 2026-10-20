#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace vista::application::lio_detail {

// One causal, bounded six-channel detector. It never edits measurements or
// timestamps. Only isolated deviations inflate R; a sustained step is accepted
// after two samples so real acceleration/rotation is not clipped to old values.
class ImuRobustifier {
public:
    using Channels = std::array<double, 6>; // gyro xyz, acceleration xyz in SI
    Channels observe(const Channels& values) {
        Channels scales{1,1,1,1,1,1};
        if (size_ >= 9) for (std::size_t axis=0;axis<6;++axis) {
            std::array<double,15> scratch{};
            for(std::size_t i=0;i<size_;++i)scratch[i]=history_[i][axis];
            std::sort(scratch.begin(),scratch.begin()+size_);
            const double median=scratch[size_/2];
            for(std::size_t i=0;i<size_;++i)scratch[i]=std::abs(history_[i][axis]-median);
            std::sort(scratch.begin(),scratch.begin()+size_);
            const double sigma=std::max(axis<3?.003:.08,1.4826*scratch[size_/2]);
            const double score=std::abs(values[axis]-median)/sigma;
            if(score>6){
                const double difference=values[axis]-median;
                // Two alternating/glitching samples are NOT evidence of a
                // sustained acceleration step. Release only a coherent change.
                if(run_[axis] && (difference*previous_difference_[axis]<=0 ||
                    std::abs(values[axis]-previous_outlier_[axis])>std::max(3*sigma,.25*std::abs(difference))))run_[axis]=0;
                if(run_[axis]<2)scales[axis]=std::min(100.0,(score/6)*(score/6));
                run_[axis]=std::min<std::size_t>(2,run_[axis]+1);
                previous_outlier_[axis]=values[axis];previous_difference_[axis]=difference;
            }else run_[axis]=0;
        }
        history_[next_]=values;next_=(next_+1)%history_.size();size_=std::min(size_+1,history_.size());
        return scales;
    }
    std::size_t size() const noexcept{return size_;}
private:
    std::array<Channels,15> history_{};
    std::array<std::size_t,6> run_{};
    Channels previous_outlier_{},previous_difference_{};
    std::size_t next_{},size_{};
};
} // namespace vista::application::lio_detail
