
// +++ b/unsupported/doc/examples/AutoDiff.cpp

#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
//#include <Eigen/src/SparseCore/SparseVector.h>
#include <Eigen/SparseCore>

/**
 * This describes the basic use of Eigen::AutoDiffScalar
 */
void basic_use_autodiff_scalar(){
   std::cout << "== basic_use_autodiff_scalar() ==" << std::endl;
   typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
   // AScalar stores a scalar and a derivative vector.
   
   // Instantiate an AutoDiffScalar variable with a normal Scalar
   double s = 0.3;
   AScalar As(s);

   // Get the value from the Instance
   std::cout << "value: " << As.value() << std::endl;

   // The derivative vector
   As.derivatives();   // gives you a reference of 
               // the contained derivative vector
   
   // Resize the derivative vector
   As.derivatives().resize(2);
   /**
    * Important note:
    * All ActiveScalars which are used in one computation must have
    * either a common derivative vector length or a zero-length
    * derivative vector.
    */

   // Set the initial derivative vector
   As.derivatives() = Eigen::VectorXd::Unit(2,0);
   std::cout << "Derivative vector : " << 
       As.derivatives().transpose() << std::endl;

   // Instantiate another AScalar
   AScalar Ab(4);
   Ab.derivatives() = Eigen::VectorXd::Unit(2,1);

   // Do the most simple calculation
   AScalar Ac = As * Ab;

   std::cout << "Result/Ac.value()" << Ac.value() << std::endl;
   std::cout << "Gradient: " << Ac.derivatives().transpose() << std::endl;
}


void test_scalar_sparse(){
  std::cout << "== autodiff_sparse() ==" << std::endl;
   // Eigen::SparseVector;
   typedef Eigen::AutoDiffScalar<Eigen::SparseVector<double> > AScalar;
   //typedef Eigen::AutoDiffScalar<Eigen::SparseMatrix<double> > AScalar;
   AScalar a(3.);
   a.derivatives().resize(2);
   a.derivatives().insert(0) = 1.;

   /**
    * Using a sparse vector as a derivative vector is always slower
    * than using a dense one unless you really have a lot of derivative
    * directions.
    * 
    * The second use is that it reveils the structure of the gradient
    * which is constant. However this only works if the computations
    * use only continuous functions, nothing like min(), max().
    *
    */
}


/**
 * Typical application of automatic differentiation
 * A templated function which can be called with active scalars
 * or normal floats/doubles.
 */
template <typename T>
T myfun(T const & a, T const & b){
   T c = pow(sin(a),2.) + pow(cos(b),2.) + 1.;
   return c;
}

void test_scalar(){
   std::cout << "== test_scalar() ==" << std::endl;
   // use with normal floats
   double a,b;
   a = 0.3;
   b = 0.5;
   double c = myfun(a,b);
   std::cout << "Result: " << c << std::endl;

   // use with AutoDiffScalar
   typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
   AScalar Aa,Ab;
   Aa.value() = 0.3;
   Ab.value() = 0.5;
   Aa.derivatives() = Eigen::VectorXd::Unit(2,0);
   Ab.derivatives() = Eigen::VectorXd::Unit(2,1);
   AScalar Ac = myfun(Aa,Ab);
   std::cout << "Result: " << Ac.value() << std::endl;
   std::cout << "Gradient: " << 
       Ac.derivatives().transpose() << std::endl;
}


/**
 * A sample function which has matrix valued arguments and
 * a scalar result. Again the function is templated so it
 * can be used with either normal floating point number
 * and AutoDiffScalar.
 */
template <typename T>
T my_matrixfun(
       Eigen::Matrix<T,Eigen::Dynamic,1> const &a,
       Eigen::Matrix<T,Eigen::Dynamic,1> const &b){
   Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> m(a.size(),a.size());
   m.setZero();
   m.diagonal() = a;
   return (b.transpose() * m * b)(0,0);
}

/**
 * Extract the gradient from my_matrixfun()
 */
void test_matrix(){
   std::cout << "== test_matrix() ==" << std::endl;
   // use with normal floats
   Eigen::VectorXd a(3),b(3);
   a.setLinSpaced(0.,1.);
   b.setLinSpaced(1.,2.);
   double c = my_matrixfun(a,b);
   std::cout << "Result: " << c << std::endl;

   // use with AutoDiffScalar
   typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
   typedef Eigen::Matrix<AScalar,Eigen::Dynamic,1> AVector;
   AVector Aa(a.size()),Ab(b.size());
   // copy value from non-active example
   for(int i=0;i<a.size();i++)Aa(i).value() = a(i);
   for(int i=0;i<b.size();i++)Ab(i).value() = b(i);
   // initialize derivative vectors
   const int derivative_num = a.size() + b.size();
   int derivative_idx = 0;
   for(int i=0;i<Aa.size();i++){
       Aa(i).derivatives() = 
          Eigen::VectorXd::Unit(derivative_num, derivative_idx);
       derivative_idx++;
   }
   for(int i=0;i<Ab.size();i++){
       Ab(i).derivatives() = 
          Eigen::VectorXd::Unit(derivative_num, derivative_idx);
       derivative_idx++;
   }

   AScalar Ac = my_matrixfun(Aa,Ab);
   std::cout << "Result: " << Ac.value() << std::endl;
   std::cout << "Gradient: " << 
       Ac.derivatives().transpose() << std::endl;
}

/**
 * Tag an twice active scalar with a given derivative direction.
 */
template <typename T>
void init_twice_active_var(T &ad,int d_num, int idx){
        // initialize derivative direction in value field of outer active variable
        ad.value().derivatives() = T::DerType::Scalar::DerType::Unit(d_num,idx);
        // initialize derivatives direction of the variable
        ad.derivatives() = T::DerType::Unit(d_num,idx);
        // initialize Hessian matrix of variable to zero
        for(int idx=0;idx<d_num;idx++){
                ad.derivatives()(idx).derivatives()  = T::DerType::Scalar::DerType::Zero(d_num);
        }
}

/**
 * Generating the gradient and the hessian from my_matrixfun
 */
void test_matrix_twice(){
   std::cout << "== test_matrix_twice() ==" << std::endl;
   // use with normal floats
   Eigen::VectorXd a(3),b(3);
   a.setLinSpaced(0.,1.);
   b.setLinSpaced(1.,2.);
   double c = my_matrixfun(a,b);
   std::cout << "Result: " << c << std::endl;

   // use with AutoDiffScalar
   typedef Eigen::Matrix<double,Eigen::Dynamic,1> inner_derivative_type;
   typedef Eigen::AutoDiffScalar<inner_derivative_type> inner_active_scalar;
   typedef Eigen::Matrix<inner_active_scalar,Eigen::Dynamic,1> outer_derivative_type;
   typedef Eigen::AutoDiffScalar<outer_derivative_type> outer_active_scalar;
   typedef Eigen::Matrix<outer_active_scalar,Eigen::Dynamic,1> AVector;
   AVector Aa(a.size()),Ab(b.size());
   // copy value from non-active example
   for(int i=0;i<a.size();i++)Aa(i).value().value() = a(i);
   for(int i=0;i<b.size();i++)Ab(i).value().value() = b(i);
   // initialize derivative vectors
   const int derivative_num = a.size() + b.size();
   int derivative_idx = 0;
   for(int i=0;i<Aa.size();i++){
       init_twice_active_var(Aa(i),derivative_num,derivative_idx);
       derivative_idx++;
   }
   for(int i=0;i<Ab.size();i++){
       init_twice_active_var(Ab(i),derivative_num,derivative_idx);
       derivative_idx++;
   }

   outer_active_scalar Ac = my_matrixfun(Aa,Ab);
   std::cout << "Result: " << Ac.value().value() << std::endl;
   std::cout << "Gradient: " << 
       Ac.value().derivatives().transpose() << std::endl;

   std::cout << "Hessian" << std::endl;
   Eigen::MatrixXd hessian(Ac.derivatives().size(),Ac.derivatives().size());
   for(int idx=0;idx<Ac.derivatives().size();idx++){
       hessian.middleRows(idx,1) = 
           Ac.derivatives()(idx).derivatives().transpose();
   }
   std::cout << hessian << std::endl;
}



int main()
{
   // basic stuff
   basic_use_autodiff_scalar();
   //test_scalar();
   //test_matrix();

   // advanced stuff
   //test_matrix_twice();
   //test_scalar_sparse();
   return 0;
}

//Eigen教程3 - 稀疏矩阵操作