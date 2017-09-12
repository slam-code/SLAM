#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>
#include <iostream>

#define sout(Xit)  {std::cout<<__LINE__<<": "<< Xit <<""<<std::endl;}
using namespace std;

/*
 * f = x1*x1 * x1*x2 + x1*x1 * x2*x2*x2*x2

how to get the gradient and the hessian by AutoDiff.

 to compute the Hessian

H =
[ d2f / (dx1 dx1) d2f / (dx1 dx2);
d2f / (dx2 dx1) d2f / (dx2 dx2) ]

 */

template<typename T>
T fun(Eigen::Matrix<T, Eigen::Dynamic, 1> const &x)
{
    T y;
    y = x(0) * x(0) * x(0) * x(1) + x(0) * x(0) * x(1) * x(1) * x(1) * x(1);
    return y;
}

int main()
{
    //normal use of fun
    {
        typedef double scalar_t;
        typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> input_t;
        input_t x(2);
        x.setConstant(1);
        scalar_t y = fun(x);
        sout(x);
        sout(y);
    }

    //autodiff use of fun
    {
        typedef Eigen::Matrix<double, Eigen::Dynamic, 1> derivative_t;
        typedef Eigen::AutoDiffScalar<derivative_t> scalar_t;
        typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> input_t;
        input_t x(2);
        x.setConstant(1);

        //set unit vectors for the derivative directions (partial derivatives of the input vector)
        x(0).derivatives().resize(2);
        x(0).derivatives()(0) = 1;
        x(1).derivatives().resize(2);
        x(1).derivatives()(1) = 1;

        scalar_t y = fun(x);

        sout(y.value());

        sout(y.derivatives());
    }
    //autodiff second derivative of fun
    {
        typedef Eigen::Matrix<double, Eigen::Dynamic, 1> inner_derivative_t;
        typedef Eigen::AutoDiffScalar<inner_derivative_t> inner_scalar_t;
        typedef Eigen::Matrix<inner_scalar_t, Eigen::Dynamic, 1> derivative_t;
        typedef Eigen::AutoDiffScalar<derivative_t> scalar_t;
        typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> input_t;
        input_t x(2);
        x(0).value() = 1;
        x(1).value() = 1;

        //set unit vectors for the derivative directions (partial derivatives of the input vector)
        x(0).derivatives().resize(2);
        x(0).derivatives().setZero();
        x(0).derivatives()(0) = 1;

        x(1).derivatives().resize(2);
        x(1).derivatives().setZero();
        x(1).derivatives()(1) = 1;

        //repeat partial derivatives for the inner AutoDiffScalar
        x(0).value().derivatives() = inner_derivative_t::Unit(2, 0);
        x(1).value().derivatives() = inner_derivative_t::Unit(2, 1);

        //set the hessian matrix to zero
        for (int idx = 0; idx < 2; idx++)
        {
            x(0).derivatives()(idx).derivatives() = inner_derivative_t::Zero(2);
            x(1).derivatives()(idx).derivatives() = inner_derivative_t::Zero(2);
        }

        scalar_t y = fun(x);

        sout(y.value().value());

        sout(y.value().derivatives());


        sout(y.derivatives()(0).value());

        sout(y.derivatives()(0).derivatives());

        sout(y.derivatives()(1).value());

        sout(y.derivatives()(1).derivatives());

    }
}