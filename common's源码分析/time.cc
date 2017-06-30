

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
