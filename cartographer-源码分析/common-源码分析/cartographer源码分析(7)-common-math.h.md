
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++
 
文件:common/math.h

/*
common/math.h文件主要实现数学计算，包括：
区间截断.求n次方.求平方.幅度角度转换.归一化.反正切值
*/

#ifndef CARTOGRAPHER_COMMON_MATH_H_
#define CARTOGRAPHER_COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace common {

// Clamps 'value' to be in the range ['min', 'max'].
//将val截取到区间min至max中.
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'. 计算base的exp次方
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.  特化,求平方
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.角度到弧度的转换. 60° -> pi/3
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.弧度到角度的转换, pi/3 -> 60°
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi]
//将角度差转换为[-pi;pi] 
template <typename T>
T NormalizeAngleDifference(T difference) {
  while (difference > M_PI) {
    difference -= T(2. * M_PI);
  }
  while (difference < -M_PI) {
    difference += T(2. * M_PI);
  }
  return difference;
}


/*
atan2 返回原点至点(x,y)的方位角，即与 x 轴的夹角，
也可以理解为计算复数 x+yi 的辐角,范围是[-pi,pi]
ATAN2(1,1) -> pi/4:以弧度表示点(1,1)的反正切值，即pi/4(0.785398)
*/
template <typename T>
T atan2(const Eigen::Matrix<T, 2, 1>& vector) {  //范围是[-pi,pi]
  return ceres::atan2(vector.y(), vector.x());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MATH_H_



```


```c++

测试代码:common/math_test.cc

#include "cartographer/common/math.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(MathTest, testPower) {
  EXPECT_EQ(0., Power(0, 42));// 0的42次方==0
  EXPECT_EQ(1., Power(0, 0)); //0^0 ==0
  EXPECT_EQ(1., Power(1, 0)); // 1^0 ==0
  EXPECT_EQ(1., Power(1, 42));//1^42==1
  EXPECT_EQ(4., Power(2, 2)); //2^2==4
}

TEST(MathTest, testPow2) {
  EXPECT_EQ(0., Pow2(0)); //0^2==0
  EXPECT_EQ(1., Pow2(1)); //1^2==1
  EXPECT_EQ(4., Pow2(2)); //2^2==4
  EXPECT_EQ(49., Pow2(7));//7^2==49
}

TEST(MathTest, testDeg2rad) {
  EXPECT_NEAR(M_PI, DegToRad(180.), 1e-9);        // 180° ==pi
  EXPECT_NEAR(2. * M_PI, DegToRad(360. - 1e-9), 1e-6);//360° ==2pi
}

TEST(MathTest, testRad2deg) {
  EXPECT_NEAR(180., RadToDeg(M_PI), 1e-9);         //pi ==180°
  EXPECT_NEAR(360., RadToDeg(2. * M_PI - 1e-9), 1e-6);//2pi ==360°
}

TEST(MathTest, testNormalizeAngleDifference) {
  EXPECT_NEAR(0., NormalizeAngleDifference(0.), 1e-9);        //0==0
  EXPECT_NEAR(M_PI, NormalizeAngleDifference(M_PI), 1e-9);    //pi==oi
  EXPECT_NEAR(-M_PI, NormalizeAngleDifference(-M_PI), 1e-9);  //-pi==-pi
  EXPECT_NEAR(0., NormalizeAngleDifference(2. * M_PI), 1e-9);   //2pi==0
  EXPECT_NEAR(M_PI, NormalizeAngleDifference(5. * M_PI), 1e-9); //5pi==pi
  EXPECT_NEAR(-M_PI, NormalizeAngleDifference(-5. * M_PI), 1e-9);//-5pi=-pi
}

}  // namespace
}  // namespace common
}  // namespace cartographer

```


本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 （推荐）
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce