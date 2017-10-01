
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:data.h


#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {


/*  
Data是针对某一类的传感器的数据的封装,
数据成员含:
1,传感器类型,包括imu，雷达，里程计
2,测量时间,time
3,Imu测量值,(imu:惯性测量单元是测量物体三轴姿态角(或角速率)以及加速度的装置。)
4,rangefinder,测距仪,
5,odometer_pose,里程计,指(如装在汽车上的)测量行程的装置。

构造函数有3个,每一类传感器对应一个构造函数.


*/
// This type is a logical union, i.e. only one type of sensor data is actually
// filled in. It is only used for time ordering sensor data before passing it
// on.
struct Data {
  enum class Type { kImu, kRangefinder, kOdometer };

  struct Imu {
    Eigen::Vector3d linear_acceleration; //线性加速度,m/s2
    Eigen::Vector3d angular_velocity;    //角速度, rad/s
  };
  
//测距仪.基本原理是向待测距的物体发射激光脉冲并开始计时，接收到反射光时停止计时。这段时间即可以转换为激光器与目标之间的距离。
  struct Rangefinder {
    Eigen::Vector3f origin; //sensor的位姿
    PointCloud ranges;      //测距点云
  };

  Data(const common::Time time, const Imu& imu) //传感器类型是imu
      : type(Type::kImu), time(time), imu(imu) {}

  Data(const common::Time time, const Rangefinder& rangefinder)//传感器类型是雷达
      : type(Type::kRangefinder), time(time), rangefinder(rangefinder) {}

  Data(const common::Time time, const transform::Rigid3d& odometer_pose)
  //传感器类型是里程计
      : type(Type::kOdometer), time(time), odometer_pose(odometer_pose) {}

  Type type;
  common::Time time;
  Imu imu;
  Rangefinder rangefinder;
  transform::Rigid3d odometer_pose;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DATA_H_

/*
rigidbody:刚体 
*/


```


```c++
测试代码:


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
