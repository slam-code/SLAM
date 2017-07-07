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
