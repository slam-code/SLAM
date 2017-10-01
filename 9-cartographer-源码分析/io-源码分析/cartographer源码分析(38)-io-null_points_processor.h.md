
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:】null_points_processor.h



#ifndef CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_

#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*
NullPointsProcessor是PointsProcessor的第十个子类(10)
空操作类。通常用于pipline的尾端，丢弃所有的点云，表示整个处理流程已经完毕。
*/

// A points processor that just drops all points. The end of a pipeline usually.
class NullPointsProcessor : public PointsProcessor {
 public:
  NullPointsProcessor() {}
  ~NullPointsProcessor() override {}
    
  //不作任何事情
  void Process(std::unique_ptr<PointsBatch> points_batch) override {}
  //返回"kFinished"状态，此操作会传递给上一个流水线。
  FlushResult Flush() override { return FlushResult::kFinished; }
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_



```

 

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
