

#include "cartographer/common/fixed_ratio_sampler.h"

namespace cartographer {
namespace common {

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio) {}

FixedRatioSampler::~FixedRatioSampler() {}

bool FixedRatioSampler::Pulse() { 

    //如果当前采样率 samples/ pulses低于固定采样率,此次事件计入采样.
  ++num_pulses_;
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    ++num_samples_;
    return true;
  }
  return false;
}

string FixedRatioSampler::DebugString() {
  return std::to_string(num_samples_) + " (" +
         std::to_string(100. * num_samples_ / num_pulses_) + "%)";
}

}  // namespace common
}  // namespace cartographer
