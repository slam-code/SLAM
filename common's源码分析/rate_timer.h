
#ifndef CARTOGRAPHER_COMMON_RATE_TIMER_H_
#define CARTOGRAPHER_COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

/*
 * RateTimer是,脉冲频率计数类,计算在一段时间内的脉冲率
 * RateTimer不可拷贝和赋值
 * RateTimer只有一个构造函数,提供时间段Duration
 * ComputeRate()返回事件脉冲率,单位hz
 * ComputeWallTimeRateRatio()返回真实时间与墙上挂钟时间的比率
 * 内部类Event封装了某个事件发生的时间点.
 * 调用一次Pulse()即产生了一次事件
 * */
namespace cartographer {
namespace common {

// Computes the rate at which pulses come in.默认模板参数是steady_clock
template <typename ClockType = std::chrono::steady_clock>
class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}

  RateTimer(const RateTimer&) = delete; //不可拷贝/不可赋值
  RateTimer& operator=(const RateTimer&) = delete;

  // Returns the pulse rate in Hz.
  double ComputeRate() const { //计算频率
    if (events_.empty()) {
      return 0.;
    }
     //事件次数除以时间即为每秒钟发生多少次事件
    return static_cast<double>(events_.size() - 1) / 
           common::ToSeconds((events_.back().time - events_.front().time));
     //最晚发生的时间-最早发生的时间 (事件产生时的真实时间)
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  double ComputeWallTimeRateRatio() const { //返回比率
    if (events_.empty()) {
      return 0.;
    }
    return common::ToSeconds((events_.back().time - events_.front().time)) / //真实时间
            //墙上挂钟时间,->调用Pulse时的系统的时间
           std::chrono::duration_cast<std::chrono::duration<double>>(        
               events_.back().wall_time - events_.front().wall_time)
               .count();
  }

  // Records an event that will contribute to the computed rate.
  void Pulse(common::Time time) { //产生一个脉冲
    events_.push_back(Event{time, ClockType::now()});
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
  struct Event {
    common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }

  std::deque<Event> events_;
  const common::Duration window_duration_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_RATE_TIMER_H_
