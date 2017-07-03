
#ifndef CARTOGRAPHER_IO_POINTS_BATCH_H_
#define CARTOGRAPHER_IO_POINTS_BATCH_H_

#include <array>
#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"

namespace cartographer {
namespace io {

// A point's color.
using Color = std::array<uint8_t, 3>;

/*
PointsBatch类是对多个点云point的抽象.这些point在由同一时刻,同一机器人坐标地点的传感器采集而得。
数据成员主要描述了point的特性.

1,time:point采集时间.
2,origin:sensor的世界坐标
3,frame_id:帧id
4,trajectory_index:轨迹线id
5,points:几何参数,vector<{x,y,z}>
6,intensities:光强
7,colors:point的rgb值
*/


// A number of points, captured around the same 'time' and by a
// sensor at the same 'origin'.
struct PointsBatch {
  PointsBatch() {
    origin = Eigen::Vector3f::Zero();
    trajectory_index = 0;
  }

  // Time at which this batch has been acquired. point被采集的时间点
  common::Time time; 

  // Origin of the data, i.e. the location of the sensor in the world at
  // 'time'. 传感器位姿
  Eigen::Vector3f origin; 

  // Sensor that generated this data's 'frame_id' or empty if this information
  // is unknown. 关键帧的id
  string frame_id;

  // Trajectory index that produced this point. 轨迹线
  int trajectory_index;

  // Geometry of the points in a metric frame. 公制，point的几何参数
  std::vector<Eigen::Vector3f> points;

  // Intensities are optional and may be unspecified. The meaning of these
  // intensity values varies by device. For example, the VLP16 provides values
  // in the range [0, 100] for non-specular return values and values up to 255
  // for specular returns. On the other hand, Hokuyo lasers provide a 16-bit
  // value that rarely peaks above 4096.
  std::vector<float> intensities; //光强度，可选项

  // Colors are optional. If set, they are RGB values.彩色rgb数字
  std::vector<Color> colors;
};

// Removes the indices in 'to_remove' from 'batch'.
// 按照to_temove中的索引,在batch中移除某些point.
void RemovePoints(std::vector<int> to_remove, PointsBatch* batch);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_BATCH_H_
