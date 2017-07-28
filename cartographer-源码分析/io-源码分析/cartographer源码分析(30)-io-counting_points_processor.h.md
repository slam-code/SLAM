
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:counting_points_processor.h

#ifndef CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

/*

PointsProcessor的第二个子类(2).
功能:记录有多少 point被输出处理

数据成员:
1),num_points_记录points数量
2),next_下一阶段的PointsProcessor.

*/
// Passes through points, but keeps track of how many points it saw and output
// that on Flush.
class CountingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "dump_num_points";
  explicit CountingPointsProcessor(PointsProcessor* next);

  static std::unique_ptr<CountingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~CountingPointsProcessor() override {}

  CountingPointsProcessor(const CountingPointsProcessor&) = delete;
  CountingPointsProcessor& operator=(const CountingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> points) override;
  FlushResult Flush() override;

 private:
  int64 num_points_;
  PointsProcessor* next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_COUNTING_POINTS_PROCESSOR_H_



```


```c++
counting_points_processor.cc

#include "cartographer/io/counting_points_processor.h"

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

//初始化是num是0
CountingPointsProcessor::CountingPointsProcessor(PointsProcessor* next)
    : num_points_(0), next_(next) {}

//与其他FromDictionary()函数统一写法
std::unique_ptr<CountingPointsProcessor>
CountingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<CountingPointsProcessor>(next);
}

//不处理points,而是将num_points_加上batch.size(),，即统计点云数据。然后直接流水线到下一PointsProcessor
void CountingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  num_points_ += batch->points.size();//相加
  next_->Process(std::move(batch));
}

//依据下一阶段的状态重置本阶段的状态。
PointsProcessor::FlushResult CountingPointsProcessor::Flush() {
  switch (next_->Flush()) {
    case FlushResult::kFinished:
      LOG(INFO) << "Processed " << num_points_ << " and finishing.";
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(INFO) << "Processed " << num_points_ << " and restarting stream.";
      num_points_ = 0; //重启,则置为0
      return FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer



```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
