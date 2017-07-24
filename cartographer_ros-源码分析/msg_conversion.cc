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

#include "cartographer_ros/msg_conversion.h"

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::transform::Rigid3d;
using ::cartographer::kalman_filter::PoseCovariance;
using ::cartographer::sensor::PointCloudWithIntensities;

/*
对frame_id做一些前处理工作,具体可见
http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
http://docs.ros.org/api/std_msgs/html/msg/Header.html
http://docs.ros.org/api/sensor_msgs/html/msg/PointField.html
*/
sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64 timestamp,
                                                   const string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;

  /*填充header*/
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp)); //时间戳
  msg.header.frame_id = frame_id;                                             //frame_id


  msg.height = 1;                                   //height 
  msg.width = num_points;                           //width


  /*填充fields*/
  msg.fields.resize(3);                             //3个数组,zyx
  msg.fields[0].name = "x";                         
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::LaserEcho& echo) {
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::LaserEcho& echo) {
  return echo.echoes[0];
}

/*
LaserScan消息的示例如下:
header: 
  seq: 101732
  stamp: 
    secs: 1487324446
    nsecs: 379890600
  frame_id: laser
angle_min: -3.12413907051
angle_max: 3.14159274101
angle_increment: 0.0174532923847
time_increment: 1.55012827463e-07
scan_time: 5.56496015633e-05
range_min: 0.15000000596
range_max: 8.0
ranges: [inf, inf, 5.64900016784668, 2.875999927520752, inf,...
intensities: [0.0, 0.0, 47.0, 47.0, 0.0, 47.0, 0.0, 0.0, 0.0,....


Header header            # Header也是一个结构体,包含了seq,stamp,frame_id,其中seq指的是扫描顺序增加的id,stamp包含了开始扫描的时间s和与开始扫描的时间ns,
                         frame_id是扫描的参考系名称.注意扫描是逆时针从正前方开始扫描的.

float32 angle_min        # 开始扫描的角度(角度)
float32 angle_max        # 结束扫描的角度(角度)
float32 angle_increment  # 每一次扫描增加的角度(角度)

float32 time_increment   # 测量的时间间隔(s)
float32 scan_time        # 扫描的时间间隔(s)

float32 range_min        # 距离最小值(m)
float32 range_max        # 距离最大值(m)

float32[] ranges         # 距离数组(长度360)
float32[] intensities    # 与设备有关,强度数组(长度360)


http://blog.csdn.net/android_ruben/article/details/55518769
*/


//函数模板是sensor_msgs::LaserScan 一类的类型
//该函数提供激光扫描数据LaserScan到 PointCloudWithIntensities 格式的转换－＞sensor/point_cloud.h 
// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>
PointCloudWithIntensities LaserScanToPointCloudWithIntensities(
    const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud; //{x,y,z}+intensity
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];      //对每一个扫描的激光 ,float时表示距离
    if (HasEcho(echoes)) {                   //有回射时,包装成点云 ,float时,为true

      const float first_echo = GetFirstEcho(echoes);//float时即原值
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {   //检查合法性，距离d必须在ｍｉｎ－ｍａｘ之间
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());//初始角度位置

        point_cloud.points.push_back(rotation *
                                     (first_echo * Eigen::Vector3f::UnitX()));//正式插入point{x,y,0}
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));//正式插入光强度值echo_intensities
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }

    }
    angle += msg.angle_increment;                                    // 重要,必须更新下一组数据的角度值
  }
  return point_cloud;
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

/*
将PointCloud转换为ros的PointCloud2格式
*/
sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud) {

  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size()); //添加head信息
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());        //stream关联到data()的地址
  for (const auto& point : point_cloud) {  //输入四个数
    stream.next(point.x());              
    stream.next(point.y());
    stream.next(point.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::PointCloud2& message) {

  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(message, "intensity")) { //有强度intensity时,
    pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
    pcl::fromROSMsg(message, pcl_point_cloud);    //先转换成pcl格式,再转换
    for (const auto& point : pcl_point_cloud) {   //对于每一个点{x,y,z}
      point_cloud.points.emplace_back(point.x, point.y, point.z);
      point_cloud.intensities.push_back(point.intensity);
    }
  } else {                                        //没有强度intensity时
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(message, pcl_point_cloud);

    // If we don't have an intensity field, just copy XYZ and fill in
    // 1.0.
    for (const auto& point : pcl_point_cloud) {
      point_cloud.points.emplace_back(point.x, point.y, point.z);
      point_cloud.intensities.push_back(1.0);
    }
  }
  return point_cloud;


}

/*
geometry_msgs::TransformStamped格式见
http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html
http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
*/
Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),// x,y,z
                 ToEigen(transform.transform.rotation));  //z,y,z,w
}

/*
http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
*/
Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) { //x,y,x
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));         //x,y,z,w
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);//x,y,z
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {//x,y,z,w
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

PoseCovariance ToPoseCovariance(const boost::array<double, 36>& covariance) {
  return Eigen::Map<const Eigen::Matrix<double, 6, 6>>(covariance.data());
}

/*
Rigid3d到ros的转换
*/
geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}
/*
ros到Rigid3d的转换
*/
geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position.x = rigid3d.translation().x();
  pose.position.y = rigid3d.translation().y();
  pose.position.z = rigid3d.translation().z();
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

}  // namespace cartographer_ros
