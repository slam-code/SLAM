
// To get an auto differentiated cost function, you must define a class with a
// templated operator() (a functor) that computes the cost function in terms of
// the template parameter T. The autodiff framework substitutes appropriate
// "jet" objects for T in order to compute the derivative when necessary, but
// this is hidden, and you should write the function as if T were a scalar type
// (e.g. a double-precision floating point number).
//
// The function must write the computed value in the last argument (the only
// non-const one) and return true to indicate success.
//
// For example, consider a scalar error e = k - x'y, where both x and y are
// two-dimensional column vector parameters, the prime sign indicates
// transposition, and k is a constant. The form of this error, which is the
// difference between a constant and an expression, is a common pattern in least
// squares problems. For example, the value x'y might be the model expectation
// for a series of measurements, where there is an instance of the cost function
// for each measurement k.
//
// The actual cost added to the total problem is e^2, or (k - x'k)^2; however,
// the squaring is implicitly done by the optimization framework.
//
// To write an auto-differentiable cost function for the above model, first
// define the object
//
//   class MyScalarCostFunction {
//     MyScalarCostFunction(double k): k_(k) {}
//
//     template <typename T>
//     bool operator()(const T* const x , const T* const y, T* e) const {
//       e[0] = T(k_) - x[0] * y[0] + x[1] * y[1];
//       return true;
//     }
//
//    private:
//     double k_;
//   };
//
// Note that in the declaration of operator() the input parameters x and y come
// first, and are passed as const pointers to arrays of T. If there were three
// input parameters, then the third input parameter would come after y. The
// output is always the last parameter, and is also a pointer to an array. In
// the example above, e is a scalar, so only e[0] is set.
//
// Then given this class definition, the auto differentiated cost function for
// it can be constructed as follows.
//
//   CostFunction* cost_function
//       = new AutoDiffCostFunction<MyScalarCostFunction, 1, 2, 2>(
//           new MyScalarCostFunction(1.0));              ^  ^  ^
//                                                        |  |  |
//                            Dimension of residual ------+  |  |
//                            Dimension of x ----------------+  |
//                            Dimension of y -------------------+
//
// In this example, there is usually an instance for each measumerent of k.
//
// In the instantiation above, the template parameters following
// "MyScalarCostFunction", "1, 2, 2", describe the functor as computing a
// 1-dimensional output from two arguments, both 2-dimensional.
//
// The autodiff cost function also supports cost functions with a
// runtime-determined number of residuals. For example:
//
//   CostFunction* cost_function
//       = new AutoDiffCostFunction<MyScalarCostFunction, DYNAMIC, 2, 2>(
//           new CostFunctionWithDynamicNumResiduals(1.0),   ^     ^  ^
//           runtime_number_of_residuals); <----+            |     |  |
//                                              |            |     |  |
//                                              |            |     |  |
//             Actual number of residuals ------+            |     |  |
//             Indicate dynamic number of residuals ---------+     |  |
//             Dimension of x -------------------------------------+  |
//             Dimension of y ----------------------------------------+
//
// The framework can currently accommodate cost functions of up to 6 independent
// variables, and there is no limit on the dimensionality of each of them.
//
// WARNING #1: Since the functor will get instantiated with different types for
// T, you must to convert from other numeric types to T before mixing
// computations with other variables of type T. In the example above, this is
// seen where instead of using k_ directly, k_ is wrapped with T(k_).
//
// WARNING #2: A common beginner's error when first using autodiff cost
// functions is to get the sizing wrong. In particular, there is a tendency to
// set the template parameters to (dimension of residual, number of parameters)
// instead of passing a dimension parameter for *every parameter*. In the
// example above, that would be <MyScalarCostFunction, 1, 2>, which is missing
// the last '2' argument. Please be careful when setting the size parameters.



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
CostFunction就是f(x)=10-x;
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
  template <typename T> bool operator()(const T* const x, T* residual) const { //adj. 残留的.必须传指针
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
                                   const vector<double*>& parameter_blocks);
                                   //或者double
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