
//--------自动微分的说明--------------

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


