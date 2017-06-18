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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

/*
ComputeCroppedProbabilityGrid计算剪裁后的grid
*/
ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid);

/*
获取选项
*/
proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* parameter_dictionary);

struct Submap : public mapping::Submap { // 继承父类
  Submap(const MapLimits& limits, const Eigen::Vector2f& origin);

  ProbabilityGrid probability_grid;
};

/*A container of Submaps. 继承抽象基类
不可拷贝/赋值
提供3个操作
1,构造函数根据 SubmapsOptions 创建一个子图
2,size()返回子图的数量,Get()返回index
3,InsertRangeData()将测量得到的距离数据插入到submap集合中

*/
class Submaps : public mapping::Submaps {
 public:
  explicit Submaps(const proto::SubmapsOptions& options);

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  const Submap* Get(int index) const override;//submaps_[index].get()
  int size() const override;                  //submaps_.size()
  void SubmapToProto(
      int index, const transform::Rigid3d& global_submap_pose,
      mapping::proto::SubmapQuery::Response* response) const override;

  // Inserts 'range_data' into the Submap collection.
  void InsertRangeData(const sensor::RangeData& range_data);

 private:
  void FinishSubmap(int index);                 //标记index对应的submap完成
  void AddSubmap(const Eigen::Vector2f& origin);// 在{x,y,0}处添加一个submap

  const proto::SubmapsOptions options_;

  std::vector<std::unique_ptr<Submap>> submaps_;
  RangeDataInserter range_data_inserter_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
