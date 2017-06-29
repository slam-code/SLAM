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

#ifndef CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
#define CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace kalman_filter {
/*
高斯分布类,构造函数是N*1的均值矩阵+N*N的协方差矩阵


*/
template <typename T, int N>
class GaussianDistribution {
 public:
  GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean,
                       const Eigen::Matrix<T, N, N>& covariance)
      : mean_(mean), covariance_(covariance) {}

  const Eigen::Matrix<T, N, 1>& GetMean() const { return mean_; }

  const Eigen::Matrix<T, N, N>& GetCovariance() const { return covariance_; }

 private:
  Eigen::Matrix<T, N, 1> mean_;       //N*1
  Eigen::Matrix<T, N, N> covariance_; //N*N
};

/*
重载+加号操作符,高斯+高斯=对应均值+均值,对应协方差+协方差,返回的是新的高斯对象
见高斯分布性质:
https://zh.wikipedia.org/wiki/%E6%AD%A3%E6%80%81%E5%88%86%E5%B8%83
*/
template <typename T, int N>
GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) {
  return GaussianDistribution<T, N>(lhs.GetMean() + rhs.GetMean(),
                                    lhs.GetCovariance() + rhs.GetCovariance());
}

/*
乘法,*运算符,矩阵*高斯分布:N*M || M*1
*/
template <typename T, int N, int M>
GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs) {
  return GaussianDistribution<T, N>(
      lhs * rhs.GetMean(),                          // N*M || M*1 -> N*1

      lhs * rhs.GetCovariance() * lhs.transpose()); // N*M ||M*M || M*N ->  N*N
}

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
