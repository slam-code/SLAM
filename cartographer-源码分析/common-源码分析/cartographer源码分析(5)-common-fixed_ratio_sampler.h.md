
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:common/fixed_ratio_sampler.h


#ifndef CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
#define CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

/*
FixedRatioSampler是频率固定的采样器类，目的是从数据流中均匀的按照固定频率采样数据
FixedRatioSampler不可拷贝,不可赋值.
成员函数提供2种操作:
Pulse()产生一个事件pulses,并且:如果计入采样samples，返回true
DebugString():以string形式输出采样率

*/
// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
class FixedRatioSampler {
 public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  bool Pulse();

  // Returns a debug string describing the current ratio of samples to pulses.
  string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  const double ratio_;

  int64 num_pulses_ = 0; //产生的脉冲次数
  int64 num_samples_ = 0;//记录的采样次数
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_


```

.


```c++
文件：fixed_ratio_sampler.cc:



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


```
...

```c++
测试代码:


#include "cartographer/common/fixed_ratio_sampler.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(FixedRatioSamplerTest, AlwaysTrue) {
  FixedRatioSampler fixed_ratio_sampler(1.);//固定采样率是1hz，每次Pulse都采集samples
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(fixed_ratio_sampler.Pulse());
  }
}

TEST(FixedRatioSamplerTest, AlwaysFalse) {
  FixedRatioSampler fixed_ratio_sampler(0.);//0 hz，不采集samples
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(fixed_ratio_sampler.Pulse());
  }
}

TEST(FixedRatioSamplerTest, SometimesTrue) {
  FixedRatioSampler fixed_ratio_sampler(0.5); //0.5hz
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(i % 2 == 0, fixed_ratio_sampler.Pulse());//每2次Pulse采集一次samples
  }
}

TEST(FixedRatioSamplerTest, FirstPulseIsTrue) {
  // Choose a very very small positive number for the ratio.
  FixedRatioSampler fixed_ratio_sampler(1e-20); //0.000000...001hz
  EXPECT_TRUE(fixed_ratio_sampler.Pulse());
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(fixed_ratio_sampler.Pulse()); //每100000000次Pulse采集一次samples
  }
}

}  // namespace
}  // namespace common
}  // namespace cartographer


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode