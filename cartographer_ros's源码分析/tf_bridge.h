/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_TF_BRIDGE_H_
#define CARTOGRAPHER_ROS_TF_BRIDGE_H_

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "tf2_ros/buffer.h"
//http://docs.ros.org/lunar/api/tf2_ros/html/c++/buffer_8h_source.html
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

/*
tf:Transform,不同坐标系之间的变换和旋转
http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup/TF
http://wiki.ros.org/tf2
http://docs.ros.org/jade/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html

TfBridge类

3个数据成员
1,tracking_frame_
2,lookup_transform_timeout_sec_
3,buffer_
*/
class TfBridge {
 public:
  TfBridge(const string& tracking_frame, double lookup_transform_timeout_sec,
           const tf2_ros::Buffer* buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge&) = delete;
  TfBridge& operator=(const TfBridge&) = delete;

/*
当在time时,返回从frame_id到tracking_frame_的变换.参数：time, frame_id
*/
  // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time'.
  std::unique_ptr<::cartographer::transform::Rigid3d> LookupToTracking(
      ::cartographer::common::Time time, const string& frame_id) const;

 private:
  const string tracking_frame_;
  const double lookup_transform_timeout_sec_;
  const tf2_ros::Buffer* const buffer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TF_BRIDGE_H_
