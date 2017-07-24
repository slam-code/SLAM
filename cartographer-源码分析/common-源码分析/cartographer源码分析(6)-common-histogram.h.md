
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++
 
 文件:common/histogram.h


#ifndef CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include <map>
#include <string>
#include <vector>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {
/*
Histogram直方图类,
提供2个操作:
1，Add()添加元素
2，ToString(buckets )以字符串的形式输出buckets个直方图信息,bin大小是篮子个数.

Histogram只有一个数据成员，用vector<float>表示

*/
class Histogram {
 public:
  void Add(float value);       //添加value,可乱序
  string ToString(int buckets) const;//分为几块

 private:
  std::vector<float> values_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_HISTOGRAM_H_



/*测试：
若bin为2:[2,1,2,4,5,5]:
Count: 6  Min: 1.000000  Max: 5.000000  Mean: 3.166667
[1.000000, 3.000000)              ##########    Count: 3 (50.000000%)   Total: 3 (50.000000%)
[3.000000, 5.000000]              ##########    Count: 3 (50.000000%)   Total: 6 (100.000000%)


若bin为3:[2,1,2,4,5,5,6,7,8]:
Count: 10  Min: 1.000000  Max: 8.000000  Mean: 4.500000
[1.000000, 3.333333)                  ######    Count: 3 (30.000000%)   Total: 3 (30.000000%)
[3.333333, 5.666667)                ########    Count: 4 (40.000000%)   Total: 7 (70.000000%)
[5.666667, 8.000000]                  ######    Count: 3 (30.000000%)   Total: 10 (100.000000%)

*/


```


```c++
测试代码histogram_le_test:


#include "cartographer/common/math.h"

#include "cartographer/common/histogram.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(Hist, toString) {
 Histogram hist;
   hist.Add(2);
   hist.Add(1);
   hist.Add(2);
   hist.Add(4);
   hist.Add(5);
   hist.Add(5);
   hist.Add(5);
   hist.Add(6);
   hist.Add(7);
   hist.Add(8);
   std::cout<<hist.ToString(3)<<std::endl;

}
输出信息:
Count: 10  Min: 1.000000  Max: 8.000000  Mean: 4.500000
[1.000000, 3.333333)                  ######    Count: 3 (30.000000%)   Total: 3 (30.000000%)
[3.333333, 5.666667)                ########    Count: 4 (40.000000%)   Total: 7 (70.000000%)
[5.666667, 8.000000]                  ######    Count: 3 (30.000000%)   Total: 10 (100.000000%)


```


本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 （推荐）
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce