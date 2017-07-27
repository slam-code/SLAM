
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:transform/transform_interpolation_buffer.h：



#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

/*
基于时间有序的变换,支持在队列中按时间顺序查找,即使变换不存在于队列中，任然支持相邻时间内的插值变换进行近似。
作用与ROS的tf2函数族类似。

数据成员：
1,deque_;
成员函数：
1,Push()
2,Has()
3,Lookup()
4,earliest_time()
5,latest_time()
6,empty()
*/
// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
    //函数,返回智能指针
  static std::unique_ptr<TransformInterpolationBuffer> FromTrajectory(
      const mapping::proto::Trajectory& trajectory);

/*
添加变换到队列尾部,当缓冲区已满时,删除队首元素
*/
  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

//返回能否在给定时间内计算的插值变换。time应在early-old之间，可以插值。
  // Returns true if an interpolated transfrom can be computed at 'time'
  bool Has(common::Time time) const;
//返回time处的变换,可插值
  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

/*
返回队列缓冲区内变换的最早时间，也就是队首元素。
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;
/*
最晚时间，也就是队尾元素
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  struct TimestampedTransform {
    common::Time time;           //发生时间
    transform::Rigid3d transform;//变换矩阵
  };

  std::deque<TimestampedTransform> deque_;
  //队列，元素是带时间戳的变换,存储了一段时间内的变换矩阵信息
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_



```
.

```c++
测试代码:transform_interpolation_buffer_test.cc


#include "cartographer/transform/transform_interpolation_buffer.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

TEST(TransformInterpolationBufferTest, testHas) {
  TransformInterpolationBuffer buffer;
  EXPECT_FALSE(buffer.Has(common::FromUniversal(50)));//false 50us
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());//
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));//false，只有1个元素。
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));//true，1个元素
  EXPECT_FALSE(buffer.Has(common::FromUniversal(75))); //false
  buffer.Push(common::FromUniversal(100), transform::Rigid3d::Identity());
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));//false，50-100才是true
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));
  EXPECT_TRUE(buffer.Has(common::FromUniversal(75)));//在50-100之间
  EXPECT_TRUE(buffer.Has(common::FromUniversal(100)));
  EXPECT_FALSE(buffer.Has(common::FromUniversal(125)));//不在
  EXPECT_EQ(common::FromUniversal(50), buffer.earliest_time());
  EXPECT_EQ(common::FromUniversal(100), buffer.latest_time());
}

TEST(TransformInterpolationBufferTest, testLookup) {
  TransformInterpolationBuffer buffer;
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());
  // The rotation needs to be relatively small in order for the 
  // interpolation to remain a z-axis rotation.
  buffer.Push(common::FromUniversal(100),
              transform::Rigid3d::Translation(Eigen::Vector3d(10., 10., 10.)) *
                  transform::Rigid3d::Rotation(
                      Eigen::AngleAxisd(2., Eigen::Vector3d::UnitZ())));
  const common::Time time = common::FromUniversal(75);
  //75在50-100之间，可以使用插值计算。
  const transform::Rigid3d interpolated = buffer.Lookup(time);
  EXPECT_THAT(
      interpolated,
      IsNearly(transform::Rigid3d::Translation(Eigen::Vector3d(5., 5., 5.)) *
                   transform::Rigid3d::Rotation(
                       Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ())),
               1e-6));
}

}  // namespace
}  // namespace transform
}  // namespace cartographer



```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
