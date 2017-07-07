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

#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {


//求概率odds的log对数
// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);


/*
将[0.1,0.9]映射为0-255之间的数
*/
// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

/*
一个独立的子图,在局部slam frame中有一个local_pose,
追踪有多少range data 添加到其中.并设置finished_probability_grid用于闭环检查

数据成员有
1,local_pose                 位姿
2,num_range_data             测量数据
3,finished_probability_grid  完成建图的概率网格

*/
// An individual submap, which has a 'local_pose' in the local SLAM frame, keeps
// track of how many range data were inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
struct Submap {
  Submap(const transform::Rigid3d& local_pose) : local_pose(local_pose) {}

  // Local SLAM pose of this submap.
  const transform::Rigid3d local_pose;//子图位姿

  // Number of RangeData inserted.
  int num_range_data = 0;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertRangeData() will change the submap.
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
};

/*
Submaps是一系列的子图,初始化以后任何阶段均有2个子图,old submap 用于配准,new submap用于next配准
,next submap变成 old submap,new new submap 用于当前配准,一直交替下去

*/
// Submaps is a sequence of maps to which scans are matched and into which scans
// are inserted.
//
// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover,
// a "new" submap gets inserted.

/*
Submaps不可拷贝/赋值,是虚基类

虚基类,没有数据成员,
成员函数 
1,matching_index()返回最后一个submap的索引,用于配准
2,insertion_indices()返回最后2个submap的索引,用于点云插入
3,Get()返回给定索引的子图
4,
AddProbabilityGridToResponse()
*/
class Submaps {
 public:
  static constexpr uint8 kUnknownLogOdds = 0;

  Submaps();
  virtual ~Submaps();

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  int matching_index() const;

  // Returns the indices of the Submap into which point clouds will
  // be inserted.
  std::vector<int> insertion_indices() const;

  // Returns the Submap with the given 'index'. The same 'index' will always
  // return the same pointer, so that Submaps can be identified by it.
  virtual const Submap* Get(int index) const = 0;

  // Returns the number of Submaps.
  virtual int size() const = 0;

  // Fills data about the Submap with 'index' into the 'response'.
  virtual void SubmapToProto(int index,
                             const transform::Rigid3d& global_submap_pose,
                             proto::SubmapQuery::Response* response) const = 0;

 protected:
  static void AddProbabilityGridToResponse(
      const transform::Rigid3d& local_submap_pose,
      const mapping_2d::ProbabilityGrid& probability_grid,
      proto::SubmapQuery::Response* response);
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
