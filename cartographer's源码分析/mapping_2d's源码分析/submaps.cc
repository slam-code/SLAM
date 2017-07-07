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

#include "cartographer/mapping_2d/submaps.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"
#include "webp/encode.h"

namespace cartographer {
namespace mapping_2d {

namespace {
/*
将probability_grid按照filename制成rgb图像
*/
void WriteDebugImage(const string& filename,
                     const ProbabilityGrid& probability_grid) {
  constexpr int kUnknown = 128;
  const mapping_2d::CellLimits& cell_limits =
      probability_grid.limits().cell_limits();
  const int width = cell_limits.num_x_cells;
  const int height = cell_limits.num_y_cells;
  std::vector<uint8_t> rgb;
  for (const Eigen::Array2i& xy_index : mapping_2d::XYIndexRangeIterator(
           probability_grid.limits().cell_limits())) {
    CHECK(probability_grid.limits().Contains(xy_index));
    const uint8_t value =
        probability_grid.IsKnown(xy_index)
            ? common::RoundToInt(
                  (1. - probability_grid.GetProbability(xy_index)) * 255 + 0)
            : kUnknown;
    rgb.push_back(value);
    rgb.push_back(value);
    rgb.push_back(value);
  }

  //rgb存储了图像的信息,并调用用WebPEncodeLosslessRGB储存在output指向的内存中.

  /*
WEBP_EXTERN(size_t) WebPEncodeLosslessRGB(const uint8_t* rgb,
                                          int width, int height, int stride,
                                          uint8_t** output);
  */
  uint8_t* output = nullptr;
  size_t output_size =
      WebPEncodeLosslessRGB(rgb.data(), width, height, 3 * width, &output);
  std::unique_ptr<uint8_t, void (*)(void*)> output_deleter(output, std::free);
  std::ofstream output_file(filename, std::ios::out | std::ios::binary);
  output_file.write(reinterpret_cast<char*>(output), output_size);//以char形式写入size大小的字符
  output_file.close();
  CHECK(output_file) << "Writing " << filename << " failed.";
}

}  // namespace

ProbabilityGrid ComputeCroppedProbabilityGrid( //剪裁
    const ProbabilityGrid& probability_grid) {
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  const double resolution = probability_grid.limits().resolution();
  const Eigen::Vector2d max =
      probability_grid.limits().max() -
      resolution * Eigen::Vector2d(offset.y(), offset.x());
  ProbabilityGrid cropped_grid(MapLimits(resolution, max, limits));
  cropped_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

proto::SubmapsOptions CreateSubmapsOptions(  //根据option创建子图
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  options.set_half_length(parameter_dictionary->GetDouble("half_length"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  options.set_output_debug_images(
      parameter_dictionary->GetBool("output_debug_images"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

//无s
Submap::Submap(const MapLimits& limits, const Eigen::Vector2f& origin) //构造函数,根据Limits和 2f创建一个submap ,坐标点是 {x,y,0}
    : mapping::Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      probability_grid(limits) {}

/*
   range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "}

*/
Submaps::Submaps(const proto::SubmapsOptions& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

void Submaps::InsertRangeData(const sensor::RangeData& range_data) {
  for (const int index : insertion_indices()) { //父类的insertion_indices()
    Submap* submap = submaps_[index].get();
    CHECK(submap->finished_probability_grid == nullptr);
    range_data_inserter_.Insert(range_data, &submap->probability_grid);
    ++submap->num_range_data;
  }
  const Submap* const last_submap = Get(size() - 1);
  if (last_submap->num_range_data == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
}

const Submap* Submaps::Get(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

int Submaps::size() const { return submaps_.size(); }

void Submaps::SubmapToProto(
    const int index, const transform::Rigid3d&,
    mapping::proto::SubmapQuery::Response* const response) const {
  AddProbabilityGridToResponse(Get(index)->local_pose,
                               Get(index)->probability_grid, response);
}

/*标记完成*/
void Submaps::FinishSubmap(int index) {
  // Crop the finished Submap before inserting a new Submap to reduce peak
  // memory usage a bit.
  Submap* submap = submaps_[index].get();
  CHECK(submap->finished_probability_grid == nullptr);
  submap->probability_grid =
      ComputeCroppedProbabilityGrid(submap->probability_grid);
  submap->finished_probability_grid = &submap->probability_grid;
  if (options_.output_debug_images()) {
    // Output the Submap that won't be changed from now on.
    WriteDebugImage("submap" + std::to_string(index) + ".webp",
                    submap->probability_grid);
  }
}

void Submaps::AddSubmap(const Eigen::Vector2f& origin) {
  if (size() > 1) {
    FinishSubmap(size() - 2);
  }
  const int num_cells_per_dimension =       // 10/0.05
      common::RoundToInt(2. * options_.half_length() / options_.resolution()) +
      1;
  submaps_.push_back(common::make_unique<Submap>( //在{x,y,0}处添加一个submap
      MapLimits(options_.resolution(),
                origin.cast<double>() +
                    options_.half_length() * Eigen::Vector2d::Ones(),
                CellLimits(num_cells_per_dimension, num_cells_per_dimension)),
      origin));
  LOG(INFO) << "Added submap " << size();
}

}  // namespace mapping_2d
}  // namespace cartographer
