
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:transform/transform.h

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_H_

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

//google采用三维 xyz:roll, pitch, yaw
  
/*
 atan2:返回一个非负的弧度值,  double atan2(double y,double x) :计算复数 x+yi 的辐角
 vec()：a vector expression of the imaginary part (x,y,z) 
 norm()： for vectors, the l2 norm of *this, for matrices the Frobenius norm。
 w()  : the w coefficient

 为何乘以2: 四元数q=[cos(θ/2),sin(θ/2)x,sin(θ/2)y,sin(θ/2)z]
*/

 
// Returns the non-negative rotation angle in radians of the 3D transformation
// 'transform'.
template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) {
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}
/*
返回3D变换yaw方向的弧度值,也就是z轴方向的弧度。

*/
// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}

/*
返回3D变换yaw方向(z轴)的弧度值
*/
// Returns the yaw component in radians of the given 3D transformation
// 'transform'.
template <typename T>
T GetYaw(const Rigid3<T>& transform) {
  return GetYaw(transform.rotation());
}

/*
根据四元数返回绕任意轴旋转的矩阵。angle-axis
*/
// Returns an angle-axis vector (a vector with the length of the rotation angle
// pointing to the direction of the rotation axis) representing the same
// rotation as the given 'quaternion'.
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<T>& quaternion) {
  Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
  // We choose the quaternion with positive 'w', i.e., the one with a smaller
  // angle that represents this orientation.
  if (normalized_quaternion.w() < 0.) { //w为负时，全部乘以-1
    // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
    normalized_quaternion.w() *= T(-1.);
    normalized_quaternion.x() *= T(-1.);
    normalized_quaternion.y() *= T(-1.);
    normalized_quaternion.z() *= T(-1.);
  }
  // We convert the normalized_quaternion into a vector along the rotation axis
  // with length of the rotation angle.
  //得到角度angle
  const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                normalized_quaternion.w());
  constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
  //scale 放大倍数。
  const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
  return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

/*
与上面函数相反的操作。
给定绕angle-axis旋转，返回四元数。
*/
// Returns a quaternion representing the same rotation as the given 'angle_axis'
// vector.
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

/*投影到2维平面xy。
具体：
1,取平移中的[dx,dy],
2,xy平面的平移角度是yaw.
*/
// Projects 'transform' onto the XY plane.
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) {
  return Rigid2<T>(transform.translation().template head<2>(),
                   GetYaw(transform));
}

/*
将2维变换转换为3维变换。
具体:
1,平移矩阵为[dx,dy,0]
2,z方向单位旋转。
*/
// Embeds 'transform' into 3D space in the XY plane.Embeds：嵌入。z轴不变。
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
  return Rigid3<T>(
      {transform.translation().x(), transform.translation().y(), T(0)},
      Eigen::AngleAxis<T>(transform.rotation().angle(),
                          Eigen::Matrix<T, 3, 1>::UnitZ()));
}

/*
序列化和反序列化的函数,实现在.cc文件

*/
// Conversions between Eigen and proto.
Rigid2d ToRigid2(const proto::Rigid2d& transform);
Eigen::Vector2d ToEigen(const proto::Vector2d& vector);
Eigen::Vector3f ToEigen(const proto::Vector3f& vector);
Eigen::Vector3d ToEigen(const proto::Vector3d& vector);
Eigen::Quaterniond ToEigen(const proto::Quaterniond& quaternion);
proto::Rigid2d ToProto(const Rigid2d& transform);
proto::Rigid2f ToProto(const Rigid2f& transform);
proto::Rigid3d ToProto(const Rigid3d& rigid);
Rigid3d ToRigid3(const proto::Rigid3d& rigid);
proto::Rigid3f ToProto(const Rigid3f& rigid);
proto::Vector2d ToProto(const Eigen::Vector2d& vector);
proto::Vector3f ToProto(const Eigen::Vector3f& vector);
proto::Vector3d ToProto(const Eigen::Vector3d& vector);
proto::Quaternionf ToProto(const Eigen::Quaternionf& quaternion);
proto::Quaterniond ToProto(const Eigen::Quaterniond& quaternion);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_H_



```


```c++
测试代码:


#include "cartographer/transform/transform.h"

#include <random>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

TEST(TransformTest, GetAngle) {
  std::mt19937 rng(42);//随机数种子
  std::uniform_real_distribution<float> angle_distribution(0.f, M_PI);
     //[0- pi],概率均匀分布
  std::uniform_real_distribution<float> position_distribution(-1.f, 1.f);
     //[-1 ,1]

  for (int i = 0; i != 100; ++i) {
    const float angle = angle_distribution(rng);
    const float x = position_distribution(rng);
    const float y = position_distribution(rng);
    const float z = position_distribution(rng);
    const Eigen::Vector3f axis = Eigen::Vector3f(x, y, z).normalized();
    //, for vectors, the l2 norm of *this, and for matrices the Frobenius norm. 
    
    //给定角度绕axis旋转，得到Rigid3f，再求Rigid3d的angle。2者理论上应该相同
    EXPECT_NEAR(angle,
                GetAngle(Rigid3f::Rotation(AngleAxisVectorToRotationQuaternion(
                    Eigen::Vector3f(angle * axis)))),
                1e-6f);
  }
}

TEST(TransformTest, GetYaw) {
  const auto rotation =
      Rigid3d::Rotation(Eigen::AngleAxisd(1.2345, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(1.2345, GetYaw(rotation), 1e-9);
  EXPECT_NEAR(-1.2345, GetYaw(rotation.inverse()), 1e-9);//四元数的逆就是共轭。
}

TEST(TransformTest, GetYawAxisOrdering) {
  //乘法顺序不能颠倒。zyx
  const auto rotation =
      Rigid3d::Rotation(Eigen::AngleAxisd(1.2345, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.4321, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(0.6789, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(1.2345, GetYaw(rotation), 1e-9);
}

TEST(TransformTest, Embed3D) {
  const Rigid2d rigid2d({1., 2.}, 0.);
  const Rigid3d rigid3d(
      Rigid3d::Translation(Eigen::Vector3d(1., 2., 3.)) *
      Rigid3d::Rotation(Eigen::Quaterniond(1., 2., 3., 4.).normalized()));
  const Rigid3d expected =
      rigid3d * Rigid3d::Translation(Eigen::Vector3d(1., 2., 0.));
  EXPECT_THAT(expected, IsNearly(rigid3d * Embed3D(rigid2d), 1e-9));
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
