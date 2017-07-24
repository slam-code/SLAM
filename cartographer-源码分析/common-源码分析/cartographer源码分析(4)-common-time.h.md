
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++
 
common/time.h


 /*
预备知识:
c++11 提供了语言级别的时间函数.包括duration和time_point
duration是时间段,指的是某单位时间上的一个明确的tick(片刻数)：
3分钟->"3个1分钟",
1.5个"1/3秒" :1.5是tick,1/3秒是时间单位

time_point是一个duration和一个epoch(起点)的组合：
2017年5月4日是"自1970,01,01"以来的126200000秒数


common/time.h主要功能是提供时间转换函数：


 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

//719162 是0001年1月1日到1970年1月1日所经历的天数
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>; //百万分之一秒,微秒,us
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  /*time_point的模板参数是UniversalTimeScaleClock,
  那为何其可以做模板参数呢：？符合std::关于clock的类型定义和static成员*/
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;//微秒,us
using Time = UniversalTimeScaleClock::time_point;  //时间点

// Convenience functions to create common::Durations.
//将秒数seconds转为c++的duration实例对象
Duration FromSeconds(double seconds);              
Duration FromMilliseconds(int64 milliseconds);     //毫秒

// Returns the given duration in seconds.
//将的duration实例对象转为 秒数 
double ToSeconds(Duration duration);               

// Creates a time from a Universal Time Scale.     
//将TUC时间(微秒)转化为c++的time_point对象
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为TUC时间,单位是us
int64 ToUniversal(Time time);                      

// For logging and unit tests, outputs the timestamp integer.
//重载<<操作符,将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time); 

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_


```


```c++
common/time.cc

 


#include "cartographer/common/time.h"

#include <string>

namespace cartographer {
namespace common {

//duration_cast是c++ 11的时间显式转换函数.
Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
    //将double类型的秒数转化为duration对象
      std::chrono::duration<double>(seconds));  
}

double ToSeconds(const Duration duration) {
//反转化,count()返回时钟周期数,ticks
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

//先构造一个临时duration对象,再将其转化为time_point对象
//Duration(ticks)调用的是UniversalTimeScaleClock的构造函数     
Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); } 


//count()返回time_point自epoch以来的时钟周期数
int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

//先将Time转化为 int64 , 再转为字符串形式
std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

//mill是ms,micro是us,先将毫秒转为以毫秒计时的duration对象,再转化为以微妙计.
common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

}  // namespace common
}  // namespace cartographer

```