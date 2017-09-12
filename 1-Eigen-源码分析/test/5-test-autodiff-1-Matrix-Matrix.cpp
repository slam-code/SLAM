#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include <iostream>

using namespace std;

#define sout(Xit)  {std::cout<<__LINE__<<": "<< Xit <<""<<std::endl;}
void f();

int main()
{
    typedef Eigen::Matrix<double, 2, 1> derivative_type;
    typedef Eigen::AutoDiffScalar<derivative_type> active_double;
    active_double d2, d3;
    double d4;

    d2 = 0;
    d4 = 0;
    d3 = d2 * d4;

    Eigen::Matrix<active_double, 4, 4> m2;
    Eigen::Matrix<active_double, 4, 1> m3;
    Eigen::Matrix<double, 4, 1> m4;

    m4 << 1, 2, 3, 4;

    m3 = m2 * m4;

    //m2 m3 是自动微分类型matrix。
    //sout(m2.value());
    //sout(m2.derived());
    //sout(m3.value());
    //sout(m3.derived());

    f();

    return (0);
}


void f()
{
    typedef Eigen::Matrix<double, 2, 1> derivative_type;
    typedef Eigen::AutoDiffScalar<derivative_type> active_double;
    double b;
    Eigen::Matrix<active_double, 4, 1> A_;
    Eigen::Matrix<active_double, 4, 1> B_;
    A_ = b * B_; //first case

    Eigen::Matrix<double, 4, 1> B;
    active_double b_;
    //  a.lazyProduct(b).
    //A_ = B * b_; //second case
    //A_=B.lazyProduct(b_);



}