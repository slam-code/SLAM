
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
counter clock wise rotation:逆时针旋转，counter：反向
Identity():单位矩阵,不同于线性代数的“单位”。语义不同，含义不同。


Rotation是指物体自身的旋转, 也就是我我们常说的旋转, 直观的理解就是我们的视野方向没有发生改变。
Orientation则是物体本身没有动, 我们的视野方向发生了旋转。


Rigid2是对二维网格旋转的封装。
效果等同于在原来的point上平移[dx,dy]，旋转θ。模板参数必须是浮点型。

提供2个操作：
1，按照指定角度θ逆时针旋转。
2，按照指定平移向量平移

静态成员函数包括：
1,Rotation()
2,Translation()
3,Identity()

Translation()指定对象的2D translation（2D平移）。
第一个参数对应X轴，第二个参数对应Y轴。默认是单位变换。　　　


[x'       [cosθ , -sinθ           [ x
 y']   =   sinθ ,  cosθ ]   *       y ]

*/
template <typename FloatType> 
//FloatType不能是int类型，不然Rotation2D的cos0==0,丢失角度信息，无法得到正确结果
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;//2行1列,p142
  using Rotation2D = Eigen::Rotation2D<FloatType>;

// 无参构造函数,平移向量为单位向量[1,0]^t,旋转角度为0,debugstring()输出:[1,0,0]
  Rigid2()
      : translation_(Vector::Identity()), rotation_(Rotation2D::Identity()) {}

 
//Rotation2D(double ): Construct a 2D counter clock wise rotation from the angle a in radian. 

//双参构造函数，给定平移向量[dx,dy]和旋转角度，进行转换变换:  
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
//同上
  Rigid2(const Vector& translation, const double rotation)//给定旋转角度θ 
      : translation_(translation), rotation_(rotation) {}

//类的静态成员函数，返回Rigid2,debugstring()是[0,0,θ ]
  static Rigid2 Rotation(const double rotation) { //给定旋转角度θ 
    return Rigid2(Vector::Zero(), rotation);
  }
//同上
  static Rigid2 Rotation(const Rotation2D& rotation) {//给定旋转矩阵,角度为θ 
    return Rigid2(Vector::Zero(), rotation);
  }
//旋转角度是单位矩阵,即θ为0 ,[dx,dy,0]
  static Rigid2 Translation(const Vector& vector) { 

    return Rigid2(vector, Rotation2D::Identity());//θ为0
  }

//静态成员函数，全为0：
  static Rigid2<FloatType> Identity() {    //返回Rigid2,[0,0,0]
    return Rigid2<FloatType>(Vector::Zero(), Rotation2D::Identity());
  }

/*为什么要加translation_.template：
参见成员模板函数的写法：
https://stackoverflow.com/questions/29754251/issue-casting-c-eigenmatrix-types-via-templates
https://stackoverflow.com/questions/12676190/how-to-call-a-template-member-function
https://stackoverflow.com/questions/4942703/why-do-i-get-an-error-trying-to-call-a-template-member-function-with-an-explicit?answertab=votes

cast()按照指定的参数类型将数据成员进行类型转换：
rigid2数据成员原本是double,转换成int可调用：
  rigid2.cast<int>()
*/
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

//返回平移向量,[dx,dy]
  const Vector& translation() const { return translation_; }
/*返回旋转矩阵:
[cosθ , -sinθ  
 sinθ ,  cosθ ] */
  Rotation2D rotation() const { return rotation_; }

//归一化角度 ,弧度[-pi;pi]
  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());//方位角θ
  }

/*
平移变换矩阵的逆矩阵与原来的平移量相同，但是方向相反。
旋转变换矩阵的逆矩阵与原来的旋转轴相同,但是角度相反。

*/
  Rigid2 inverse() const {//求逆公式，由[dx',dy']得到[dx,dy]
    const Rotation2D rotation = rotation_.inverse(); //取负,得到-θ
    const Vector translation = -(rotation * translation_);//得到[dx,dy]
    return Rigid2(translation, rotation); //返回一个新2维网格的point
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

 private: //数据成员总是先旋转再平移，translation_表达的是在旋转θ后的方向上，做增量变换

//2行1列 的矩阵.p142 平移变换 [dx',dy'].注意,此处是增量变换,即deltax=x,deltay=y
  Vector translation_;

  Rotation2D rotation_;//Eigen::Rotation2D,方向角,旋转变换 [θ]

/*
Rotation2D介绍：
This class is equivalent to a single scalar representing a counter clock wise rotation as a single angle in radian. It provides some additional features such as the automatic conversion from/to a 2x2 rotation matrix.

Rotation2D 是二维旋转里根据逆时针旋转θ角而生成的旋转矩阵:
[cosθ , -sinθ     
 sinθ ,  cosθ ]

  */
};

/*
定义 * 乘法操作符:2个Rigid2相乘,得到第三个Rigid2,（等效于连续变换2次）
实现细节：
1，先计算经过lhs变换的point，计算经过rhs变换的point
2，角度相加，（等效于旋转矩阵相乘）
A*B.矩阵.先对B旋转,再加上delta

*/
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

/*旋转矩阵rigid乘以point,得到point：C=A*B B是point,A是旋转矩阵,
C是point结果,实现：
1，先对point[x0,y0]旋转，得到[x0',y0'].
2，[x0',y0']再加上rigid自身平移的deltax,deltay.得到point[x1,y1]

1,2不能颠倒顺序。why？：
*/
template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
  /*
Vector2 Eigen::Rotation2D< Scalar >::operator*  ( const Vector2 &   vec ) const
:
Applies the rotation to a 2D vector

  */
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
预备概念：

四元数,通常用quaternion来计算3D物体的旋转角度，与Matrix相比，quaternion更加高效，
占用的储存空间更小，此外也更便于插值。在数学上，quaternion表示复数w+xi+yj+zk，其中i,j,k都是虚数单位。

3维矩阵本身没有平移,但是可以借助4元数获得类似结果,(齐次坐标)

AngleAxis.Represents a 3D rotation as a rotation angle around an arbitrary 3D axis.
Combined with MatrixBase::Unit{X,Y,Z}, AngleAxis can be used to easily mimic Euler-angles. Here is an example:

*/




/*
Rigid3是三维网格变换
该类含义2个数据成员：

Vector translation_;
Quaternion rotation_;

一)3个构造函数分别初始化平移矩阵和旋转矩阵。

二)3个静态成员函数分别按照给定方式旋转.
Rotation(),Translation(),Identity(
)
三)

*/

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>; //3行1列的矩阵 
  using Quaternion = Eigen::Quaternion<FloatType>;//四元数,见游戏引擎架构p156
  using AngleAxis = Eigen::AngleAxis<FloatType>;  //坐标角度

//默认[1,0,0]和[1,0,0,0]
  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
//提供平移矩阵和旋转矩阵
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
//同上
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

//静态成员函数.平移为0
  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }
//平移为0
  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }
//不旋转
  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());//[x,y,z,w]为[0,0,0,1]
  }

//平移为0,不旋转
  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }

//类型转换
  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

//获取数据成员
  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

//求逆
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

//乘法操作Rigid3*Rigid3,得到Rigid3
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

//rigid3*Vector,得到Vector
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



/*
二维旋转:游戏引擎架构p142
公式推导： 
http://blog.csdn.net/zhouxuguang236/article/details/31820095
http://blog.csdn.net/csxiaoshui/article/details/65446125
https://zh.wikipedia.org/zh-hans/%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5

三维旋转:
http://blog.csdn.net/soilwork/article/details/1447346
http://www.cnblogs.com/tgycoder/p/5103966.html

可以把一个3D旋转转换为绕基本坐标轴的旋转，即绕三个坐标值x、y、z的旋转。 

一）绕X轴的旋转：x不变。围绕x坐标画圆。yoz平面

点P(x,y,z)绕x轴旋转θ角得到点P’(x’,y’,z’)。由于是绕x轴进行的旋转，因此x坐标保持不变，y和z组成的yoz（o是坐标原点）平面上进行的是一个二维的旋转

x′=x 
y′=ycosθ−zsinθ 
z′=ysinθ+zcosθ 

二）绕Y轴旋转：Y坐标保持不变，ZOX平面
x′=xcosθ+zsinθ 
y′=y 
z′=−xsinθ+zcosθ

三）绕Z轴旋转：Z坐标保持不变，xoy平面

x′=xcosθ-ysinθ 
y′=xsinθ+ycosθ 
z′=z 
*/