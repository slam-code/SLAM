
#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_TEST_HELPERS_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_TEST_HELPERS_H_

#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/transform/rigid_transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {

/*
本文件用于辅助gtest

Eigen::Translation:Represents an homogeneous transformation in a N dimensional space.
*/

//
template <typename T>
Eigen::Transform<T, 2, Eigen::Affine> ToEigen(const Rigid2<T>& rigid2) {
  return Eigen::Translation<T, 2>(rigid2.translation()) * rigid2.rotation();
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> ToEigen(const Rigid3<T>& rigid3) {
  return Eigen::Translation<T, 3>(rigid3.translation()) * rigid3.rotation();
}
/*gmock模拟对象行为*/
MATCHER_P2(IsNearly, rigid, epsilon,
           string(string(negation ? "isn't" : "is", " nearly ") +
                  rigid.DebugString())) {
  return ToEigen(arg).isApprox(ToEigen(rigid), epsilon);
}

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_TEST_HELPERS_H_
