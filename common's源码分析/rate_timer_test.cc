/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/rate_timer.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(RateTimerTest, ComputeRate) {
  RateTimer<> rate_timer(common::FromSeconds(1.));//默认模板参数是steady_clock,设置时间段是1s
  common::Time time = common::FromUniversal(42);  // 时间点,42us
  for (int i = 0; i < 100; ++i) {                 //每隔0.1s产生一次事件,产生100次
    rate_timer.Pulse(time);
    time += common::FromSeconds(0.1);
  }
  EXPECT_NEAR(10., rate_timer.ComputeRate(), 1e-3);//频率应该等于10次每s
}

struct SimulatedClock { /*模拟的时间clock类*/
  using rep = std::chrono::steady_clock::rep;
  using period = std::chrono::steady_clock::period;
  using duration = std::chrono::steady_clock::duration;
  using time_point = std::chrono::steady_clock::time_point;
  static constexpr bool is_steady = true;

  static time_point time;
  static time_point now() noexcept { return time; }
};

SimulatedClock::time_point SimulatedClock::time;

TEST(RateTimerTest, ComputeWallTimeRateRatio) {
  common::Time time = common::FromUniversal(42);                 //时间点 42us
  RateTimer<SimulatedClock> rate_timer(common::FromSeconds(1.)); //时间段是1s
  for (int i = 0; i < 100; ++i) {                                //100次事件,0.1s一次.
    rate_timer.Pulse(time);
    time += common::FromSeconds(0.1);                           //加0.1s,代表事件发生的时间(调用了Pulse)
    SimulatedClock::time +=                                     //模仿的时间,加0.05s,代表流失了0.05秒.
        std::chrono::duration_cast<SimulatedClock::duration>(
            std::chrono::duration<double>(0.05));
  }
  EXPECT_NEAR(2., rate_timer.ComputeWallTimeRateRatio(), 1e-3); //0.1/0.05==2
}

}  // namespace
}  // namespace common
}  // namespace cartographer
