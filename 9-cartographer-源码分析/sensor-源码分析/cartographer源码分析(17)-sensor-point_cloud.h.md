
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:sensor/point_cloud.h


#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

/*
点云数据是指在一个三维坐标系统中的一组向量的集合。{p1,p2,p3,...}
这些向量通常以X,Y,Z三维坐标的形式表示p:{x,y,z}，
而且一般主要用来代表一个物体的外表面形状。
除{x,y,z}可以代表的几何位置信息之外，
点云数据还可以表示一个点的RGB颜色，灰度值，深度，分割结果等。

Eg..Pi={Xi, Yi, Zi,…….}表示空间中的一个点，
则Point Cloud={P1, P2, P3,…..Pn}表示一组点云数据。

cartographer的PointCloud是由Vector3f组成的vector。
PointCloudWithIntensities则是由点云和光线强度组成的struct类。


*/
namespace cartographer {
namespace sensor {

typedef std::vector<Eigen::Vector3f> PointCloud;//vector，元素是3*1f

struct PointCloudWithIntensities { //点云+光线强度,{x,y,z}+intensity
  PointCloud points; //3*1的vector
  std::vector<float> intensities; 
};

// Transforms 'point_cloud' according to 'transform'. 根据三维网格参数转换点云
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);


/*去掉z轴区域外的点云,返回一个新的点云*/
// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);

//序列化
// Converts 'point_cloud' to a proto::PointCloud.
proto::PointCloud ToProto(const PointCloud& point_cloud);

//反序列化
// Converts 'proto' to a PointCloud.
PointCloud ToPointCloud(const proto::PointCloud& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_



```
.

```
文件: sensor/point_cloud.cc


#include "cartographer/sensor/point_cloud.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {


/*. 
功能：根据3D变换，转换点云,
返回：转换后的新点云结果，原点云未变。

*/
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  PointCloud result;                                //vector,元素是3*1f
  result.reserve(point_cloud.size());               //分配内存
  for (const Eigen::Vector3f& point : point_cloud) { 
    result.emplace_back(transform * point);         //C=A*B //A是Rigid3f,B是3*1f
  }
  return result;
}


/*
功能：按范围丢弃点云，去掉z轴区域外的点云
返回：新的符合要求的点云。
*/
PointCloud Crop(const PointCloud& point_cloud, const float min_z,
                const float max_z) {
  PointCloud cropped_point_cloud;
  for (const auto& point : point_cloud) {
    if (min_z <= point.z() && point.z() <= max_z) {//只保留在minz~maxz之间的点云
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}

//序列化
proto::PointCloud ToProto(const PointCloud& point_cloud) {
  proto::PointCloud proto;
  for (const auto& point : point_cloud) {
    proto.add_x(point.x());
    proto.add_y(point.y());
    proto.add_z(point.z());
  }
  return proto;
}

//反序列化
PointCloud ToPointCloud(const proto::PointCloud& proto) {
  PointCloud point_cloud;
  const int size = std::min({proto.x_size(), proto.y_size(), proto.z_size()});
  point_cloud.reserve(size);//最小，否则以下语句有bug
  for (int i = 0; i != size; ++i) {
    point_cloud.emplace_back(proto.x(i), proto.y(i), proto.z(i));
  }
  return point_cloud;
}

}  // namespace sensor
}  // namespace cartographer


```


```c++
测试代码: point_cloud_test.cc



#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(PointCloudTest, TransformPointCloud) {
  PointCloud point_cloud;
  point_cloud.emplace_back(0.5f, 0.5f, 1.f);  //构造一个点云{0.5,0.5,1}
  point_cloud.emplace_back(3.5f, 0.5f, 42.f); //{3.5,0.5,42}

  //调用static Rigid2 Rotation(const double rotation) 
  const PointCloud transformed_point_cloud = TransformPointCloud(
      point_cloud, transform::Embed3D(transform::Rigid2f::Rotation(M_PI_2))); 

/*绕z轴逆时针旋转 pi/2:
[x',y',x']=[]*[x,y,z]
化简后： x=-y，y=x，z=z
*/
  EXPECT_NEAR(-0.5f, transformed_point_cloud[0].x(), 1e-6);
  EXPECT_NEAR(0.5f, transformed_point_cloud[0].y(), 1e-6);
  EXPECT_NEAR(-0.5f, transformed_point_cloud[1].x(), 1e-6);
  EXPECT_NEAR(3.5f, transformed_point_cloud[1].y(), 1e-6);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer



```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
