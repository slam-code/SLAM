
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:min_max_range_filtering_points_processor.h

#ifndef CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*

PointsProcessor的第一个子类(1).
功能:继承自PointsProcessor点云虚基类.距离过滤器,L2 距离不在Min-Max范围内的都过滤掉.
数据成员:
1),min_range_ 最小范围
2),max_range_ 最大范围
3),PointsProcessor* const next_:完成本轮Processor,接下来进行下一次Processor.
*/

// Filters all points that are farther away from their 'origin' as 'max_range'
// or closer than 'min_range'.
class MinMaxRangeFiteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "min_max_range_filter";

   
  MinMaxRangeFiteringPointsProcessor(double min_range, double max_range,
                                     PointsProcessor* next);

  //从.lua文件加载min_range和max_range,如:0-30 
  static std::unique_ptr<MinMaxRangeFiteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~MinMaxRangeFiteringPointsProcessor() override {}//覆盖

  MinMaxRangeFiteringPointsProcessor(
      const MinMaxRangeFiteringPointsProcessor&) = delete;
  MinMaxRangeFiteringPointsProcessor& operator=(
      const MinMaxRangeFiteringPointsProcessor&) = delete;

//点云处理,删除[min,max]以外的point，并把数据传递到下一轮Processor处理。
  void Process(std::unique_ptr<PointsBatch> batch) override;

  FlushResult Flush() override;
  //调用next_->Flush();父类调用Flush(),但在运行时才会决定是否调用子类的Flush()

 private:
  const double min_range_;
  const double max_range_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_



```


```c++
min_max_range_filtering_points_processor.cc


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
      //assets_writer_backpack_2d.lua 1-60
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


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
