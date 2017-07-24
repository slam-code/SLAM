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

#include "cartographer/common/make_unique.h"

#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/tf_bridge.h"

/*
tf:Transform,不同坐标系之间的变换和旋转
http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup/TF
http://wiki.ros.org/tf2
*/
namespace cartographer_ros {

TfBridge::TfBridge(const string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time, const string& frame_id) const

     {
  ::ros::Duration timeout(lookup_transform_timeout_sec_);                  //至多等待超时时间
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;//没用,需要追踪的关键帧id,用make_unique代替了.
  try {

/*
lookup返回的是geometry_msgs::TransformStamped 
http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
lookupTransform:　http://docs.ros.org/jade/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html

lookupTransform():Get the transform between two frames by frame ID.
Parameters:
target_frame  The frame to which data should be transformed
source_frame  The frame where the data originated
time  The time at which the value of the transform is desired. (0 will get the latest)
timeout How long to block before failing
Returns:
The transform between the frames

*/

    const ::ros::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
    const ::ros::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {              //如果ft_time大于 给定的时间,说明新数据已经达到
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = ::ros::Duration(0.);                     //不等待
    }
    return ::cartographer::common::make_unique<
        ::cartographer::transform::Rigid3d>(ToRigid3d(buffer_->lookupTransform(
        tracking_frame_, frame_id, requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;

}

}  // namespace cartographer_ros
