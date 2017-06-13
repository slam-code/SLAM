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

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

void RemovePoints(std::vector<int> to_remove, PointsBatch* batch) {
  std::sort(to_remove.begin(), to_remove.end(), std::greater<int>()); //降序排列

  /*
batch->points是vector,index 是要移除的vector元素的索引

  */
  for (const int index : to_remove) {
    batch->points.erase(batch->points.begin() + index); //移除point
    if (!batch->colors.empty()) {
      batch->colors.erase(batch->colors.begin() + index); //point对应的rgb
    }
    if (!batch->intensities.empty()) {
      batch->intensities.erase(batch->intensities.begin() + index); //point对应的光强度
    }
  }
}

}  // namespace io
}  // namespace cartographer
