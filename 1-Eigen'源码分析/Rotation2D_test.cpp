#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace Eigen;
using namespace std;

#define sout(Xit)  {std::cout<<__LINE__<<":\n"<< Xit <<"\n"<<std::endl;}
//using Rotation2D = Eigen::Rotation2D<FloatType>;
int main()
{
    /*返回旋转矩阵:
[cosθ , -sinθ  
 sinθ ,  cosθ ] */
    Rotation2D<float> a(M_PI_2);
    sout(a.toRotationMatrix());

    a=Rotation2Df(M_PI/3);
    sout(a.toRotationMatrix());

    sout(a.inverse().toRotationMatrix());

    sout(a.angle()<<"=="<< M_PI/3);

    sout(a.inverse().angle()<<"=="<<-M_PI/3);

    sout(a.Identity().toRotationMatrix());


    //typedef Matrix<double,2,1> Vector2d;
    Vector2f  b;
    b<<1,2;
    sout(b);
    sout(a*b);

}

//g++ hello1.cpp 即可。不用指定include路径。