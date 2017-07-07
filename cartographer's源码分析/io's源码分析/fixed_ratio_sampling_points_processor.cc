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

#include "cartographer/io/fixed_ratio_sampling_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

/*
利用参数构造一个智能指针指向FixedRatioSamplingPointsProcessor的对象
*/
std::unique_ptr<FixedRatioSamplingPointsProcessor>
FixedRatioSamplingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {

  const double sampling_ratio(dictionary->GetDouble("sampling_ratio")); //sparse_pose_graph.lua 0.3,0.03
  CHECK_LT(0., sampling_ratio) << "Sampling ratio <= 0 makes no sense.";
  CHECK_LT(sampling_ratio, 1.) << "Sampling ratio >= 1 makes no sense.";
  return common::make_unique<FixedRatioSamplingPointsProcessor>(sampling_ratio,
                                                                next);

}

/*
构造函数用采样率 sampling_ratio和PointsProcessor* next初始化
*/
FixedRatioSamplingPointsProcessor::FixedRatioSamplingPointsProcessor(
    const double sampling_ratio, PointsProcessor* next)
    : sampling_ratio_(sampling_ratio),
      next_(next),
      sampler_(new common::FixedRatioSampler(sampling_ratio_)) {}

/*
核心函数,该函数不断的自己调用自己,
有几个为了提高效率的地方:
Process()传递的在智能指针,指向堆上的PointsBatch点集,理论上该点集组成的vector是不断增加的.


*/
void FixedRatioSamplingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  std::vector<int> to_remove;                         //保存索引
  for (size_t i = 0; i < batch->points.size(); ++i) { //points是vector
    if (!sampler_->Pulse()) {                         //传感器没有产生一个脉冲
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));//虚函数,调用子类的Process(),也就是这个函数本身
}

PointsProcessor::FlushResult FixedRatioSamplingPointsProcessor::Flush() {
  switch (next_->Flush()) {
    case PointsProcessor::FlushResult::kFinished: //完成
      return PointsProcessor::FlushResult::kFinished;

    case PointsProcessor::FlushResult::kRestartStream://重启采样
      sampler_ =
          common::make_unique<common::FixedRatioSampler>(sampling_ratio_);
      return PointsProcessor::FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
