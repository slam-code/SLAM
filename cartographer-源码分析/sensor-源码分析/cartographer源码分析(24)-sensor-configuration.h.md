
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:sensor/configuration.h


#ifndef CARTOGRAPHER_SENSOR_CONFIGURATION_H_
#define CARTOGRAPHER_SENSOR_CONFIGURATION_H_

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

//从sensor配置文件解析sensor的数据参数。主要是sensor到机器坐标的转换
// Creates the configuration for a singular sensor from 'parameter_dictionary'.
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

//求得多个sensor的配置的集合。
// Creates the mapping from frame_id to Sensors defined in
// 'parameter_dictionary'.
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

//系统是否支持某一传感器。
// Returns true if 'frame_id' is mentioned in 'sensor_configuration'.
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration);

//将sensor采集的data经过3d坐标变换为机器坐标。
// Returns the transform which takes data from the sensor frame to the
// tracking frame.
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_CONFIGURATION_H_



```
```
文件:sensor/configuration.cc
```

#include "cartographer/sensor/configuration.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {
/*
assets_writer_backpack_2d.lua

Sensor定义：
  message Sensor {
  string frame_id = 2; //'frame_id' of the sensor.
  Rigid3d transform = 3;
  }
*/
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration::Sensor sensor;
  sensor.set_frame_id(parameter_dictionary->GetString("frame_id"));

  *sensor.mutable_transform() = transform::ToProto(transform::FromDictionary(
      parameter_dictionary->GetDictionary("transform").get()));
  return sensor;
}

/*
Configuration定义：
message Configuration {
    repeated Sensor sensor = 15;
}
*/
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration configuration;
  for (auto& sensor_parameter_dictionary :
       parameter_dictionary->GetArrayValuesAsDictionaries()) {
    *configuration.add_sensor() =
        CreateSensorConfiguration(sensor_parameter_dictionary.get());
  }
  return configuration;
}

/*
判断系统是否支持某一传感器id。
*/
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return true;
    }
  }
  return false;
}

/*

将sensor坐标转换为机器坐标。以便于跟踪计算
*/
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return transform::ToRigid3(sensor.transform());
    }
  }
  LOG(FATAL) << "No configuration found for sensor with frame ID '" << frame_id
             << "'.";
}

}  // namespace sensor
}  // namespace cartographer



```c++
测试代码:


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
