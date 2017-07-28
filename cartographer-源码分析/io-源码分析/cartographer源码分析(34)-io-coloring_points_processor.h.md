
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:coloring_points_processor.h


#ifndef CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*
  {
      action = "color_points",
      frame_id = "horizontal_laser_link",
      color = { 255., 0., 0. },
    },
    {
      action = "color_points",
      frame_id = "vertical_laser_link",
      color = { 0., 255., 0. },
    },

ColoringPointsProcessor是PointsProcessor的第六子类(6).
功能: 用固定的Color填充PointsBatch的Color分量。

数据成员:
1),color_：rgb值
2),frame_id_:只有相同的id才填充Color，color一般为[255,0,0],[0,255,0]
2),next_下一阶段的PointsProcessor.

*/

// Colors points with a fixed color by frame_id.
class ColoringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "color_points";

  ColoringPointsProcessor(const Color& color, const string& frame_id,
                          PointsProcessor* next);

  static std::unique_ptr<ColoringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ColoringPointsProcessor() override{};

  ColoringPointsProcessor(const ColoringPointsProcessor&) = delete;
  ColoringPointsProcessor& operator=(const ColoringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const Color color_;
  const string frame_id_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_


```


```c++
coloring_points_processor.cc



#include "cartographer/io/coloring_points_processor.h"

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
/*
根据assets_writer_backpack_2d.lua配置:.lua文件概览:
-- Now we recolor our points by frame and write another batch of X-Rays. It
-- is visible in them what was seen by the horizontal and the vertical
-- laser.
    {
      action = "color_points",
      frame_id = "horizontal_laser_link",
      color = { 255., 0., 0. },
    },
    {
      action = "color_points",
      frame_id = "vertical_laser_link",
      color = { 0., 255., 0. },
    },
FromDictionary()根据.lua配置文件获取frame_id下color的{r,g,b}
*/


std::unique_ptr<ColoringPointsProcessor>
ColoringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
/*
frame_id:horizontal_laser_link或者vertical_laser_link
color:{ 255., 0., 0. },或者{ 0., 255., 0. },
*/
  const string frame_id = dictionary->GetString("frame_id");
  const std::vector<double> color_values =
      dictionary->GetDictionary("color")->GetArrayValuesAsDoubles();
  const Color color = {{static_cast<uint8_t>(color_values[0]),
                        static_cast<uint8_t>(color_values[1]),
                        static_cast<uint8_t>(color_values[2])}};
  return common::make_unique<ColoringPointsProcessor>(color, frame_id, next);
}

ColoringPointsProcessor::ColoringPointsProcessor(const Color& color,
                                                 const string& frame_id,
                                                 PointsProcessor* const next)
    : color_(color), frame_id_(frame_id), next_(next) {}

/*
只对相同的frame_id_处理：着色
*/
void ColoringPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->frame_id == frame_id_) {
    batch->colors.clear();
    for (size_t i = 0; i < batch->points.size(); ++i) {
      batch->colors.push_back(color_);//用color 填充batch的colors
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ColoringPointsProcessor::Flush() {
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
