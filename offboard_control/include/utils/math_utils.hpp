/*
 * filename: math_utils.hpp 
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    
 */


#pragma once 

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Eigen>

namespace math_utils
{

/* Get factorial of input positive integer.Output type is optional. */
template <typename OutputType = int>
OutputType factorial(int n)
{
    if (n <= 1)
        return static_cast<OutputType>(1);
    
    int out = 1;
    for (int i = 2; i <= n; i++)
        out *= i;

    return static_cast<OutputType>(out);
}

/* Get derivatives polynomial basis [1 coeff*t ... (coeff*t)^N]^T */
template <unsigned N>
Eigen::Matrix<double, N + 1, 1> getDPolyBasis(double t, unsigned dorder = 0, double coeff = 1.0)
{
    Eigen::Matrix<double, N + 1, 1> out;
    out.setZero();

    for (int i = dorder; i <= N; i++)
        out(i) = factorial<double>(i) / factorial<double>(i - dorder) * 
                    std::pow(t, double(i - dorder)) * std::pow(coeff, double(i));
    
    return out;
}
}
