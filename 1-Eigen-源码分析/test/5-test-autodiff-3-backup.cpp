#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>
#include <iostream>

using namespace std;
/*
 * f = x1*x1 * x1*x2 + x1*x1 * x2*x2*x2*x2

how to get the gradient and the hessian by AutoDiff.

 to compute the Hessian

H =
[ d2f / (dx1 dx1) d2f / (dx1 dx2);
d2f / (dx2 dx1) d2f / (dx2 dx2) ]

 */
#define sout(Xit)  {std::cout<<__LINE__<<": "<< Xit <<""<<std::endl;}

template<typename T>
T fun(Eigen::Matrix<T, Eigen::Dynamic, 1> const &x)
{
    T y;
    //y = x1*x1 * x1*x2 + x1*x1 * x2*x2*x2*x2
    y = pow(x(0), 3) * x(1) + pow(x(0), 2) * pow(x(1), 4);

    //sout(x);
    sout(y);

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
        x.setConstant(1);//Sets all coefficients in this expression to \a value.

        //set unit vectors for the derivative directions (partial derivatives of the input vector)
        x(0).derivatives().resize(2);
        x(0).derivatives()(0) = 1;
        x(1).derivatives().resize(2);
        x(1).derivatives()(1) = 1;

        scalar_t y = fun(x);
        sout(*(int *) x.data());
        //sout(x);
        sout(y.value());

        sout(y.derivatives());
    }
    return 0;
}