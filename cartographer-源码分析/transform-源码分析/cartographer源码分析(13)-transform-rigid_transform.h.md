
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:transform/rigid_transform.h


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
Identity():单位矩阵,不同于线性代数的“单位”。在不同的语义下，含义会不同。


Rotation是指物体自身的旋转, 也就是普通的旋转,
我们的视野方向没有发生改变，而是物体本身发生了旋转。

Orientation则是物体本身没有动, 我们的视野方向发生了旋转。


Rigid2是对二维网格旋转与平移的封装,对应论文公式(1):
Rigid2表现为先将vector[x0,y0]旋转θ角度得到[x',y']，然后再叠加[dx,dy]得到[x1,y1]
即:
x1=x'+dx,
y1=y'+dy,


模板参数必须是浮点型。(int类型为错误用法)

提供2个操作：
1，按照指定角度θ逆时针旋转。此时dx,dy=0
2，按照指定平移向量dx,dy平移. 此时θ=0

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

//双参构造函数，给定平移向量[dx,dy]和旋转角度0，进行旋转变换:  
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
//同上，给定旋转角度θ,double是弧度值。
  Rigid2(const Vector& translation, const double rotation)
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
Rigid2数据成员原本是double,对象rigid2转换成float可调用：
rigid2.cast<float>()
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
    return common::NormalizeAngleDifference(rotation().angle());
    //方位角θ.以弧度计。
  }

/*
平移变换矩阵的逆矩阵与原来的平移量相同，但是方向相反。
旋转变换矩阵的逆矩阵与原来的旋转轴相同,但是角度相反。

Rotation2D重载了operator*(maxtrix<sclar,2,1>).故可以使用乘法。
乘以rotation：[dx,dy]是旋转了θ以后的[dx,dy]
而求逆是反方向旋转θ，故需要求[dx,dy]在旋转θ之前的方向的投影。
translation加负号的原因：x1=x'+dx，所以x'=x1-dx
*/
  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse(); //取负,得到-θ。
    const Vector translation = -(rotation * translation_);//得到[-dx’,-dy‘]
    return Rigid2(translation, rotation); 
    //返回一个新2维网格的point.[-dx’,-dy‘,-θ]
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

//2行1列 的矩阵.平移向量[dx,dy]
  Vector translation_;

//旋转角度。旋转指的是Vector的旋转。
  Rotation2D rotation_;//Eigen::Rotation2D,方向角,旋转变换 [θ]

/*
Rotation2D介绍：
Rotation2D 是二维旋转里根据逆时针旋转θ角而生成的旋转矩阵:
[cosθ , -sinθ     
 sinθ ,  cosθ ]

  */
};

/*
定义 * 乘法操作符:2个Rigid2相乘,得到第三个Rigid2,
等效于连续变换2次
实现细节：
1，最终的[dx'',dy'']等于lhs在rhs方向上的投影[dx',dy']加上rhs自身的[dx,dy]
2，角度相加(等效于旋转矩阵相乘)

*/
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

/*
参数：旋转平移rigid,向量vector
返回值：向量vector。
等效于对某个2维向量先自身旋转，再叠加rigid的[dx,dy]。
实现：
1，先对vector[x0,y0]自身旋转0，得到[x',y'].
2，[x',y']再加上rigid自身的dx,dy.得到vector[x1,y1]
即先旋转自身，再叠加[dx.dy]
x1=x'+dx
y1=y'+dy

*/
template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
  /*
Rotation2D的乘法原型：
Vector2 Eigen::Rotation2D< Scalar >::operator*  ( const Vector2 &   vec ) const
:
Applies the rotation to a 2D vector

  */
}

//定义 << 输出运算符
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
占用的储存空间更小，此外也更便于插值。在数学上，quaternion表示复数w+xi+yj+zk，其中i,j,k都是虚数单位。四元数也可以表示为[w,iv].其中虚数v=[x,y,z]

3维矩阵本身没有平移,但是可以借助4元数获得类似结果,(齐次坐标)

3D旋转可以分解为绕x,y,z三个方向的旋转的叠加。
而Eigen的AngleAxis表示3D旋转绕任意轴axis的旋转。
并且与MatrixBase::Unit{X,Y,Z}组合,AngleAxis 能非常容易的得到欧垃角。
*/




/*
Rigid3是三维网格变换
该类含义2个数据成员：

Vector translation_;
Quaternion rotation_;

1),3个构造函数分别初始化平移矩阵和旋转矩阵。

2),3个静态成员函数分别按照给定方式旋转.
   Rotation(),Translation(),Identity()

3),

*/

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>; //3行1列的矩阵 
  using Quaternion = Eigen::Quaternion<FloatType>;//四元数,见游戏引擎架构p156
  using AngleAxis = Eigen::AngleAxis<FloatType>;  //坐标角度

//构造函数，默认[1,0,0]和[1,0,0,0]。
  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
//构造函数，提供平移向量[dx,dy,dz]和旋转四元数
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
//构造函数，提供平移向量[dx,dy,dz]和绕坐标轴旋转量
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

//静态成员函数.[dx,dy,dz]为0,只绕坐标轴旋转。
  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }
//静态成员函数, 只旋转，不平移。
  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }
//不旋转,只平移[dx,dy,dz]
  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());//[x,y,z,w]为[0,0,0,1]
  }

//[0,0,0]和[1,0,0,0]。
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

/*求逆,即逆方向旋转和平移。
细节：
1),四元数的逆是共轭。
2),求[-dx‘,-dy’,-dz‘]

*/
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
  Vector translation_;//x,y,z方向上的平移向量[dx,dy,dz]
  Quaternion rotation_;//四元数。旋转表达。
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

游戏引擎架构,p126,姿态角（Euler角）左手:
pitch是围绕X轴旋转，也叫做俯仰角
yaw是围绕Y轴旋转，也叫偏航角
roll是围绕Z轴旋转，也叫翻滚角

右手笛卡尔坐标系的 绕x, y 和 z轴的旋转分别叫做 roll, pitch 和 yaw 旋转。
或者:google采用三维 xyz:roll, pitch, yaw.
x-, y- 和 z-轴的旋转分别叫做 roll, pitch 和 yaw 旋转
绕x轴:θx 是 roll 角，和右手螺旋的方向相反（在yz平面顺时针）
绕y轴:θy 是 pitch 角，和右手螺旋的方向相反（在zx平面顺时针）。
绕z轴:θz 是yaw 角，和右手螺旋的方向相反（在xy平面顺时针）。

ref:https://zh.wikipedia.org/wiki/%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5
*/
// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

/*

从assets_writer_backpack_2d.lua取得如：

XY_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0., -math.pi / 2., 0., },
}

*/
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
 
3D旋转的简单推导：
可以把一个3D旋转转换为绕基本坐标轴的旋转，即绕三个坐标值x、y、z的旋转。

一）绕X轴的旋转：x不变。围绕x坐标画圆。yoz平面，方向是顺时针(与右手方向相反)

点P(x,y,z)绕x轴旋转θ角得到点P’(x’,y’,z’)。由于是绕x轴进行的旋转，因此x坐标保持不变，y和z组成的yoz（o是坐标原点）平面上进行的是一个二维的旋转
p=[1,0,0],q=[0,1,0],r=[0,0,1]
三个坐标轴单位向量p,q,r旋转后：
p'=[1, 0,    0]
q'=[0,cosθ,-sinθ]
r'=[0,sinθ,cosθ]
所以:
x′=x 
y′=ycosθ−zsinθ 
z′=ysinθ+zcosθ 

二）绕Y轴旋转：Y坐标保持不变，ZOX平面
p'=[cosθ, 0, sinθ]
q'=[  0,  1,  0]
r'=[-sinθ,0,cosθ]
所以：
x′=xcosθ+zsinθ 
y′=y 
z′=−xsinθ+zcosθ

三）绕Z轴旋转：Z坐标保持不变，xoy平面
p'=[sinθ,-cosθ,0]
q'=[sinθ, cosθ,0]
r'=[0,0,1]
所以:
x′=xcosθ-ysinθ 
y′=xsinθ+ycosθ 
z′=z 
*/



```


```c++
本人写的测试代码:rigid_transform_le2_grid2_test.cc


#include "cartographer/transform/rigid_transform.h"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include <iostream>
namespace cartographer {
namespace transform {
#define sout(Xit)  {std::cout<<__LINE__<<" "<< Xit <<""<<std::endl;}
namespace {


TEST(Rigid2s, rigid_1) {
  Rigid2<double> r1; 
  sout(r1);                 //[1,0,0]
  sout(r1.Identity());      //[0,0,0]
  sout(r1.Rotation(3.14/2));//[0,0,1.57]

  sout(r1.Translation({1,2}));//[1,2,0],静态成员函数，与对象无关
  sout(Rigid2<int>::Translation({1,2}))//同上

  sout(r1.inverse());      //[-1,-0,-0]
  sout(r1.inverse().inverse());//[1,0,0]

  sout(r1*r1);              //[2,0,0]
  Eigen::Matrix<double, 2, 1> mat{1,2};
  sout(r1*mat);             //[2,2] ：[1,0]+[1,2]
  //sout((r1*mat)(1,0));//Eigen矩阵索引是()而不是[]
  
 }

TEST(Rigid2s, rigid_2) {
  Rigid2<double> r1({1,2},M_PI_2); 
  sout(M_PI_2)             //1.57
  sout(r1);                //[1,2,1.57] ，pi/2对应的弧度是1.57
  sout(r1.Identity());     //[0,0,0]
  sout(r1.Rotation(3.14/2));//[0,0,1.57]静态成员函数，与对象无关
  
  //sout(Rigid2<int>::Rotation(3.14/2));//同上
  sout(r1.Translation({1,2}));//[1,2,0]


  sout(r1);              //[1,2,1.57]
  sout(r1.inverse());        //[-2,1,-1.57]
  sout(r1.inverse().inverse());

  Rigid2<double> r2({2,2},M_PI_2); 
  sout(r2.cast<float>()); //[2.0,2.0,1.0],成员模板函数cast()
  

  Rigid2<double> r3({3,2},3.2*2);
  sout(r3);                    //[3,2,6.4]
  sout(r3.normalized_angle()); //0.117

  
}

TEST(Rigid2s, rigid_3) {

  Rigid2<double> r1({1,2},M_PI_2); 
  Rigid2<double> r2({2,2},M_PI_2); 
  Rigid2<double> r4({2,2},M_PI_2); 

  sout(r1);                     //[1,2,1.57]
  sout(r1*r1);                 //[-1,3,3.14]
  sout(r1*r2);                 //[-1,4,3.14]

  Eigen::Matrix<double, 2, 1> mat{1,1};
  Eigen::Matrix<double, 2, 1> matd{1,1};


  sout(r1.rotation().matrix ()); //[0,-1;1,0]
  sout(r1.rotation() * mat ); //[-1,1]
  sout(r1.translation());     //[1,2] 
  sout(mat);                  //[1,1]
  sout(r1*mat); //[0,3].  




  sout(r4.rotation().matrix (   ) );//[0,-1;1,0]
  sout(r4.rotation() * matd );      //[-1,1] 
  sout(r4.translation());           //[2,2]
  sout(matd); //[1,1]
  sout(r4*matd); //[1,3].    
  }
}

}  // namespace transform
}  // namespace cartographer



```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
