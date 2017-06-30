

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
