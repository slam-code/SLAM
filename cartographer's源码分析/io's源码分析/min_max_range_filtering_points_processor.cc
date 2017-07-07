
#include "cartographer/io/min_max_range_filtering_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

//从.lua文件加载min_range和max_range,如:0-30
std::unique_ptr<MinMaxRangeFiteringPointsProcessor>
MinMaxRangeFiteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<MinMaxRangeFiteringPointsProcessor>(
      dictionary->GetDouble("min_range"), dictionary->GetDouble("max_range"), //trajectory_builder_2d.lua:0,30
      next);
}

/*
构造函数,传递min ,max2个范围参数和 PointsProcessor* next指针

*/
MinMaxRangeFiteringPointsProcessor::MinMaxRangeFiteringPointsProcessor(
    const double min_range, const double max_range, PointsProcessor* next)
    : min_range_(min_range), max_range_(max_range), next_(next) {}

void MinMaxRangeFiteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch)

     {
  std::vector<int> to_remove;    //待移除的索引
  for (size_t i = 0; i < batch->points.size(); ++i) {
    //计算sensor到远点的l2距离
    const float range = (batch->points[i] - batch->origin).norm(); 
    if (!(min_range_ <= range && range <= max_range_)) {        
      //不在给定范围内,加入移除队列.
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));//完成本轮过滤,接下来进行下一次过滤.即模拟流水线操作.
}

PointsProcessor::FlushResult MinMaxRangeFiteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
