// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A simple example of using the Ceres minimizer.
//
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// automatic differentiation.

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
/*
http://blog.sciencenet.cn/blog-3276147-1023387.html
CostFunction就是导函数定义的CostFunctor为 f(x)=10-x;
然后在problem环节中，使用AutoDiffCostFunction生成的CostFunction 为1/2* ||f(x)||^2。
因为优化框架会默认进行二范数化。

编写一个g(x)=10-x的残差方程。
第一步：利用ceres库的模板，创建CostFunctor；f(x)=10-x;
必须要编写一个重载()运算，而且必须使用模板类型，所有的输入参数和输出参数都要使用T类型。只有用模板包装时才能自动微分*/

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const { //adj. 残留的
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

/*第二步：建立Problem，把CostFunctor转换为用于ceres库解算的残差块形式
          使用class AutoDiffCostFunction(对f(x)自动求导)
          problem.AddResidualBlock（添加到残差块）
*/
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);

  /*
函数原型:  ResidualBlockId AddResidualBlock(CostFunction* cost_function,
                                   LossFunction* loss_function,
                                   const vector<double*>& parameter_blocks);//或者double
*/
  problem.AddResidualBlock(cost_function, NULL, &x);

/*
第三步：声明解算器、控制器
             Solver::Optionsoptions;
             options.linear_solver_type=ceres::DENSE_QR;
*/
  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;

/*
第四步：解算过程报告输出
             Solver::Summarysummary;
             Solve(options,&problem,&summary);
*/
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
/*
初始值是0.5 (10 - 0.5)^2=45.125,如下: 

iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  4.512500e+01    0.00e+00    9.50e+00   0.00e+00   0.00e+00  1.00e+04        0    3.35e-05    6.78e-04
   1  4.511598e-07    4.51e+01    9.50e-04   9.50e+00   1.00e+00  3.00e+04        1    7.26e-04    1.49e-03
   2  5.012552e-16    4.51e-07    3.17e-08   9.50e-04   1.00e+00  9.00e+04        1    2.46e-05    1.53e-03
Ceres Solver Report: Iterations: 3, Initial cost: 4.512500e+01, Final cost: 5.012552e-16, Termination: CONVERGENCE
x : 0.5 -> 10
从上面的输出信息中可以看出，经过三次迭代计算，求得的x值为10时可以取得最小值

接下来我们分析下main函数中的代码。

  第2行代码为Google的log库，详细内容请参考Google log库的相关说明。

  第5、6行为定义了求解未知数的初值，初值设置为0.5。 以前是5

  第9行，声明一个Problem对象，用于求解。

  第12行，声明一个残差方程，CostFunction通过模板类AutoDiffCostFunction来进行构造，
  第一个模板参数为残差对象，也就是最开始写的那个那个带有重载()运算符的结构体，
  第二个模板参数为残差个数，第三个模板参数为未知数个数，最后参数是结构体对象。

  第14行，将观测值和残差方程加入Problem对象中，如果有多个观测值，都需要加进去，这点可以看后面的示例。

  第16~19行，定义一个求解选项，里面主要包括对方程线性化的方式，迭代次数等，具体参考官网帮助文档。

  第20行，定义一个求解结果报告。

  第21行，调用Solve函数进行求解，第一个参数就是求解选项，第二个参数为Problem指针，第三个参数为求解报告指针。

  第23行，输出求解报告信息。

  第24行，输出求解前的初值和求解后的值。

  http://blog.csdn.net/liminlu0314/article/details/16808239
*/