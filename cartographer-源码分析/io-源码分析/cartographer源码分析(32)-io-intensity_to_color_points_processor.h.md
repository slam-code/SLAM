
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:intensity_to_color_points_processor.h


#ifndef CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"




namespace cartographer {
namespace io {
/*
  {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 4095.,
    },

PointsProcessor的第四个子类(4).
功能:将光强度转换为color

Intensity,图像亮度
IntensityToColorPointsProcessor：处理强度到color point 的强度变换类
成员函数执行转换:('intensity' - min ) / (max - min) * 255 

*/
class IntensityToColorPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "intensity_to_color";

  // Applies ('intensity' - min ) / (max - min) * 255 and color the point grey
  // with this value for each point that comes from the sensor with 'frame_id'.
  // If 'frame_id' is empty, this applies to all points.
  IntensityToColorPointsProcessor(float min_intensity, float max_intensity,
                                  const string& frame_id,
                                  PointsProcessor* next);

  static std::unique_ptr<IntensityToColorPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~IntensityToColorPointsProcessor() override{};

  IntensityToColorPointsProcessor(const IntensityToColorPointsProcessor&) =
      delete;
  IntensityToColorPointsProcessor& operator=(
      const IntensityToColorPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const float min_intensity_;
  const float max_intensity_;
  const string frame_id_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_


```


```c++
intensity_to_color_points_processor.cc



#include "cartographer/io/intensity_to_color_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<IntensityToColorPointsProcessor>
IntensityToColorPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const string frame_id =
      dictionary->HasKey("frame_id") ? dictionary->GetString("frame_id") : "";
  const float min_intensity = dictionary->GetDouble("min_intensity");
  const float max_intensity = dictionary->GetDouble("max_intensity");
  return common::make_unique<IntensityToColorPointsProcessor>(
      min_intensity, max_intensity, frame_id, next);
}

IntensityToColorPointsProcessor::IntensityToColorPointsProcessor(
    const float min_intensity, const float max_intensity,
    const string& frame_id, PointsProcessor* const next)
    : min_intensity_(min_intensity),
      max_intensity_(max_intensity),
      frame_id_(frame_id),
      next_(next) {}

void IntensityToColorPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  if (!batch->intensities.empty() &&
      (frame_id_.empty() || batch->frame_id == frame_id_)) {
    batch->colors.clear();
    for (const float intensity : batch->intensities) {
      const uint8_t gray =
          cartographer::common::Clamp(
              (intensity - min_intensity_) / (max_intensity_ - min_intensity_),
              0.f, 1.f) *
          255;
      batch->colors.push_back(Color{{gray, gray, gray}});
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult IntensityToColorPointsProcessor::Flush() {
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
