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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/proto/probability_grid.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
/*
2维的概率网格
数据成员有
1,limits_ 网格的边界限制
2,cells_  网格总数
3,max_x_,max_y_,min_x_,min_y_分别是网格坐标的极小极大值
4,vector<int> update_indices_ 网格中需要更新的index
*/
// Represents a 2D grid of probabilities.
class ProbabilityGrid {
 public:

//根据limits构造一个二维网格
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue),//调用vector<int>(num,val);
        max_x_(0),
        max_y_(0),
        min_x_(limits_.cell_limits().num_x_cells - 1),
        min_y_(limits_.cell_limits().num_y_cells - 1) {}

//根据proto文件构造一个概率网格
  explicit ProbabilityGrid(const proto::ProbabilityGrid& proto)
      : limits_(proto.limits()),
        cells_(),
        update_indices_(proto.update_indices().begin(),
                        proto.update_indices().end()),   //调用vector<int>v2(v1.begin(),v1.end())
        max_x_(proto.max_x()),
        max_y_(proto.max_y()),
        min_x_(proto.min_x()),
        min_y_(proto.min_y()) {
    cells_.reserve(proto.cells_size());
    for (const auto cell : proto.cells()) {
      CHECK_LE(cell, std::numeric_limits<uint16>::max());// < 65536*2
      cells_.push_back(cell);                            //cells_是vector<uint16>
    }
  }

  // Returns the limits of this ProbabilityGrid.
  const MapLimits& limits() const { return limits_; }

/*
probability_values.h
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;// 32 768

根据update_indices_的索引值,从后往前更新
*/
  // Starts the next update sequence.
  void StartUpdate() {
    while (!update_indices_.empty()) {
      //DCHECK_GE:debug check
      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      //.back()是最新的需要更新的index值,cells_[index] 比kUpdateMarker大?
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

/*
在没有概率时,根据给定的p和index设置概率,
*/
  // Sets the probability of the cell at 'xy_index' to the given 'probability'.
  // Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& xy_index, const float probability) {
    uint16& cell = cells_[GetIndexOfCell(xy_index)];
    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);//概率为0时才设置p
    cell = mapping::ProbabilityToValue(probability);  //概率p[0,1]->[1,32767]
    UpdateBounds(xy_index);//可能是新添加的网格
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'xy_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // StartUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& xy_index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);// 相等?
    const int cell_index = GetIndexOfCell(xy_index);
    uint16& cell = cells_[cell_index];
    if (cell >= mapping::kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell_index);
    cell = table[cell];
    DCHECK_GE(cell, mapping::kUpdateMarker);
    UpdateBounds(xy_index);
    return true;
  }

/*
根据给定的index返回cell对应的概率p,p的范围是[0.1,0.9]
*/
  // Returns the probability of the cell with 'xy_index'.
  float GetProbability(const Eigen::Array2i& xy_index) const {
    if (limits_.Contains(xy_index)) {
      return mapping::ValueToProbability(cells_[GetIndexOfCell(xy_index)]);// [1,32767]->[0.1,0.9]
    }
    return mapping::kMinProbability;
  }

  // Returns the probability of the cell containing the point ('x', 'y').[0.1,0.9]
  float GetProbability(const double x, const double y) const {
    return GetProbability(limits_.GetXYIndexOfCellContainingPoint(x, y));
  }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& xy_index) const {
    return limits_.Contains(xy_index) && cells_[GetIndexOfCell(xy_index)] !=
                                             mapping::kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    *offset = Eigen::Array2i(min_x_, min_y_);
    *limits = CellLimits(std::max(max_x_, min_x_) - min_x_ + 1,
                         std::max(max_y_, min_y_) - min_y_ + 1);
  }

/*
扩展Limits的边界,用以包含(x,y)
*/
  // Grows the map as necessary to include 'x' and 'y'. This changes the meaning
  // of these coordinates going forward. This method must be called immediately
  // after 'StartUpdate', before any calls to 'ApplyLookupTable'.
  void GrowLimits(const double x, const double y) {
    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetXYIndexOfCellContainingPoint(x, y))) {
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;

      
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));

      
      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;


      std::vector<uint16> new_cells(new_size,
                                    mapping::kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;         //copy新的cells_
      limits_ = new_limits;       //copy新的Limits
      min_x_ += x_offset;
      min_y_ += y_offset;
      max_x_ += x_offset;
      max_y_ += y_offset;
    }
  }

/*
序列化
*/
  proto::ProbabilityGrid ToProto() const {
    proto::ProbabilityGrid result;
    *result.mutable_limits() = cartographer::mapping_2d::ToProto(limits_);
    result.mutable_cells()->Reserve(cells_.size());
    for (const auto cell : cells_) {
      result.mutable_cells()->Add(cell);
    }
    result.mutable_update_indices()->Reserve(update_indices_.size());
    for (const auto update : update_indices_) {
      result.mutable_update_indices()->Add(update);
    }
    result.set_max_x(max_x_);
    result.set_max_y(max_y_);
    result.set_min_x(min_x_);
    result.set_min_y(min_y_);
    return result;
  }

 private:

  // Returns the index of the cell at 'xy_index'.返回xy对应的cells索引
  int GetIndexOfCell(const Eigen::Array2i& xy_index) const {
    CHECK(limits_.Contains(xy_index)) << xy_index;
    return limits_.cell_limits().num_x_cells * xy_index.y() + xy_index.x();
  }

/*
更新边界,why?:可能添加了新的网格
*/
  void UpdateBounds(const Eigen::Array2i& xy_index) {
    min_x_ = std::min(min_x_, xy_index.x());
    min_y_ = std::min(min_y_, xy_index.y());
    max_x_ = std::max(max_x_, xy_index.x());
    max_y_ = std::max(max_y_, xy_index.y());
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker.最高位是更新标记 uint:[0,65535]
  std::vector<int> update_indices_;

  // Minimum and maximum cell coordinates of known cells to efficiently compute
  // cropping limits.
  int max_x_;
  int max_y_;
  int min_x_;
  int min_y_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
