#include <iostream>

#include "Eigen/Core"
#include "Eigen/Geometry"
using Eigen::MatrixXd;
int main()
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;
}

//g++ hello1.cpp 即可。不用指定include路径。