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

#ifndef CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include <map>
#include <string>
#include <vector>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {
/*
直方图类,
提供2个操作:
Add()添加元素
ToString(buckets )以字符串的形式输出buckets个直方图信息,bin大小是篮子个数.
数据成员是vector<float>表示;

若bin为2:[2,1,2,4,5,5]:
Count: 6  Min: 1.000000  Max: 5.000000  Mean: 3.166667
[1.000000, 3.000000)	          ##########	Count: 3 (50.000000%)	Total: 3 (50.000000%)
[3.000000, 5.000000]	          ##########	Count: 3 (50.000000%)	Total: 6 (100.000000%)


若bin为3:[2,1,2,4,5,5,6,7,8]:
Count: 10  Min: 1.000000  Max: 8.000000  Mean: 4.500000
[1.000000, 3.333333)	              ######	Count: 3 (30.000000%)	Total: 3 (30.000000%)
[3.333333, 5.666667)	            ########	Count: 4 (40.000000%)	Total: 7 (70.000000%)
[5.666667, 8.000000]	              ######	Count: 3 (30.000000%)	Total: 10 (100.000000%)

*/
class Histogram {
 public:
  void Add(float value);			 //添加value,可乱序
  string ToString(int buckets) const;//分为几块

 private:
  std::vector<float> values_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_HISTOGRAM_H_
