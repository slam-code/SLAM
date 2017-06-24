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

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace transform {

/*
几个名词细微的区别 :

transformation:变换
translation:平移
rotation:旋转
scaling:缩放
counter clock wise rotation:逆时针旋转
 
Rotation是指物体自身的旋转, 也就是我我们常说的旋转, 直观的理解就是我们的视野方向没有发生改变。
Orientation则是物体本身没有动, 我们的视野方向发生了旋转。

二维旋转:游戏引擎架构p142
推导
https://zhidao.baidu.com/question/521843032083852005.html
http://blog.csdn.net/zhouxuguang236/article/details/31820095
http://blog.csdn.net/csxiaoshui/article/details/65446125
https://zh.wikipedia.org/zh-hans/%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5

Rigid2是二维
Translation()指定对象的2D translation（2D平移）。
第一个参数对应X轴，第二个参数对应Y轴。默认是单位变换。　　　
Identity():单位矩阵

[x'       [cosθ , -sinθ           [ x
 y']   =   sinθ ,  cosθ ]   *       y ]

*/
template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;//2行1列,p142
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2()                                      //默认2行1列,使用单位矩阵初始化,即变换前后是一样的,debugstring:[1,0,0]
      : translation_(Vector::Identity()), rotation_(Rotation2D::Identity()) {}

 
//Rotation2D(double ): Construct a 2D counter clock wise rotation from the angle a in radian. 

//根据给定的矩阵参数转换: 给定 平移矩阵 和 旋转矩阵 
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}

  Rigid2(const Vector& translation, const double rotation)//给定旋转角度θ 
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) { //给定旋转角度θ 
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {//给定旋转矩阵,角度θ 
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) { //旋转矩阵是单位矩阵,即θ为0 ,[x,y,0]
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() {             //返回矩阵,[0,0,0]
    return Rigid2<FloatType>(Vector::Zero(), Rotation2D::Identity());
  }

//https://stackoverflow.com/questions/29754251/issue-casting-c-eigenmatrix-types-via-templates
//https://stackoverflow.com/questions/12676190/how-to-call-a-template-member-function
//https://stackoverflow.com/questions/4942703/why-do-i-get-an-error-trying-to-call-a-template-member-function-with-an-explicit?answertab=votes
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }//平移矩阵,[x,y]

  Rotation2D rotation() const { return rotation_; }//旋转矩阵 []

  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());//方位角θ ,弧度[-pi;pi]
  }

/*
平移变换矩阵的逆矩阵与原来的平移量相同，但是方向相反。
旋转变换矩阵的逆矩阵与原来的旋转轴相同,但是角度相反。

*/
  Rigid2 inverse() const {//求逆,公式
    const Rotation2D rotation = rotation_.inverse(); //取负
    const Vector translation = -(rotation * translation_);//得到[x,y]
    return Rigid2(translation, rotation); //返回一个新2维网格
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;//2行1列 的矩阵.p142 平移变换 [x',y'].注意,此处是增量变换,即deltax=x,deltay=y
  Rotation2D rotation_;//Eigen::Rotation2D,方向角,旋转变换 [θ]

  /*
Rotation2D 是二维旋转里根据逆时针旋转θ角而生成的旋转矩阵:
                [cosθ , -sinθ     
Rotation2D=      sinθ ,  cosθ ]

  */
};

//定义 * 乘法操作符:2个平移旋转相乘,得到第三个平移旋转矩阵,A*B.矩阵.先对B旋转,再加上delta
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

// 旋转矩阵乘以point,得到point,
//C=A*B B是point,A是旋转矩阵,C是point结果: 先对point旋转,再加上平移的deltax,deltay
template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

//定义 << 输出运算符
// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

//模板特化
using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

/*
,Rigid3是三维.三维网格
http://blog.csdn.net/soilwork/article/details/1447346
http://www.cnblogs.com/tgycoder/p/5103966.html

旋转矩阵3d基础知识:
https://zh.wikipedia.org/wiki/%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5
Quaternion.四元数,通常用quaternion来计算3D物体的旋转角度，与Matrix相比，quaternion更加高效，
占用的储存空间更小，此外也更便于插值。在数学上，quaternion表示复数w+xi+yj+zk，其中i,j,k都是虚数单位

AngleAxis.Represents a 3D rotation as a rotation angle around an arbitrary 3D axis.
Combined with MatrixBase::Unit{X,Y,Z}, AngleAxis can be used to easily mimic Euler-angles. Here is an example:

*/

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>; //3行1列的矩阵 ->3维矩阵本身没有平移,但是可以借助4元数获得类似结果,(齐次坐标)
  using Quaternion = Eigen::Quaternion<FloatType>;//四元数,见游戏引擎架构p156
  using AngleAxis = Eigen::AngleAxis<FloatType>;  

  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

//乘法操作,得到A*B矩阵
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

//输出运算符
// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

//特化
using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

/*
http://blog.csdn.net/yuzhongchun/article/details/22749521

游戏引擎架构,p126,姿态角（Euler角）常用:
pitch是围绕X轴旋转，也叫做俯仰角
yaw是围绕Y轴旋转，也叫偏航角
roll是围绕Z轴旋转，也叫翻滚角

或者:

x-, y- 和 z-轴的旋转分别叫做 roll, pitch 和 yaw 旋转

google采用三维 xyz:roll, pitch, yaw

绕x轴:θx 是 roll 角，和右手螺旋的方向相反（在yz平面顺时针）:
绕y轴:θy 是 pitch 角，和右手螺旋的方向相反（在zx平面顺时针）。
绕z轴:θz 是yaw 角，和右手螺旋的方向相反（在xy平面顺时针）。

*/
// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

/*

coefficient-wise operations系数操作

the Array class provides an easy way to perform coefficient-wise operations,
which might not have a linear algebraic meaning,
such as adding a constant to every coefficient in the array or 
multiplying two arrays coefficient-wise.
*/