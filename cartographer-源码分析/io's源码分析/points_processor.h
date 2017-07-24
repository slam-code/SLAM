
#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

/*
PointsProcessor 点云虚基类,提供一种批量处理points的抽象接口,不可拷贝/赋值
2个抽象接口:
1),Process()负责对PointsBatch进行处理
2),Flush()刷新.


*/
// A processor in a pipeline. It processes a 'points_batch' and hands it to the
// next processor in the pipeline.
class PointsProcessor {
 public:
  enum class FlushResult {
    kRestartStream,
    kFinished,
  };

  PointsProcessor() {}
  virtual ~PointsProcessor() {} //必须为虚函数,不然子类无法正确析构.

  PointsProcessor(const PointsProcessor&) = delete;
  PointsProcessor& operator=(const PointsProcessor&) = delete;

  // Receive a 'points_batch', process it and pass it on.
  virtual void Process(std::unique_ptr<PointsBatch> points_batch) = 0;//纯虚函数

  // Some implementations will perform expensive computations and others that do
  // multiple passes over the data might ask for restarting the stream.
  virtual FlushResult Flush() = 0;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
