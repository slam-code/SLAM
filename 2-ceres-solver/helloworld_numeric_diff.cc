
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// numeric differentiation.

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
/*
③例3.2和3.3 numeric derivatives（数值微分）和Analytic Derivatives（解析求导）
第一步，当无法使用模板来创建costfunctor时，

第二步，建立problem时，使用NumericDiffCostFunction进行求导

*/
// A cost functor that implements the residual r = 10 - x.
struct CostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // numeric differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 1> (new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
