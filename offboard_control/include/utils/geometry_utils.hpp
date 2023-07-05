/*
 * filename: geometry_utils.hpp 
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    
 */


#pragma once 

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Eigen>

namespace geo_utils
{
Eigen::Matrix3d vec3d2Rot(const Eigen::Vector3d &dp);
Eigen::Quaterniond rpy2Quat(const Eigen::Vector3d &rpy);
Eigen::Vector3d quat2RPY(const Eigen::Quaterniond &quat);
Eigen::Vector3d quat2RPY(const Eigen::Vector4d &quat);
Eigen::Matrix3d quat2Rot(const Eigen::Vector4d &quat);
Eigen::Matrix3d rpy2Rot(const Eigen::Vector3d &rpy);
Eigen::Vector3d Rot2rpy(const Eigen::Matrix3d &rot);
Eigen::Matrix3d getSkewSymmMat(const Eigen::Vector3d &v);
Eigen::Vector3d rpy2AngVel(const Eigen::Vector3d &rpy, 
                           const Eigen::Vector3d &rpydt);
template<unsigned Dim>
typename std::vector<Eigen::Matrix<double, Dim, 1>>::iterator 
closest_it(std::vector<Eigen::Matrix<double, Dim, 1>> &points, 
        const Eigen::Matrix<double, Dim, 1> &point);
template<unsigned Dim>
Eigen::Matrix<double, Dim, 1> closest(const std::vector<Eigen::Matrix<double, Dim, 1>> &points, 
                                     const Eigen::Matrix<double, Dim, 1> &point);

/* Use vec as rotated x-axis,and set roll to 0. */
Eigen::Matrix3d vec3d2Rot(const Eigen::Vector3d &dp)
{
    double roll = 0;
    double pitch = atan2(-dp(2), sqrt(dp(0) * dp(0) + dp(1) * dp(1)));
    double yaw = atan2(dp(1), dp(0));

    Eigen::Quaterniond qx(cos(roll / 2.0), sin(roll / 2.0), 0, 0);
    Eigen::Quaterniond qy(cos(pitch / 2.0), 0, sin(pitch / 2.0), 0);
    Eigen::Quaterniond qz(cos(yaw / 2.0), 0, 0, sin(yaw / 2.0));

    return Eigen::Matrix3d(qz * qy * qx);
}

/* Transform RPY angles to quaternion. */
Eigen::Quaterniond rpy2Quat(const Eigen::Vector3d &rpy)
{
    Eigen::Quaterniond qx(cos(rpy(0) / 2.0), sin(rpy(0) / 2.0), 0, 0);
    Eigen::Quaterniond qy(cos(rpy(1) / 2.0), 0, sin(rpy(1) / 2.0), 0);
    Eigen::Quaterniond qz(cos(rpy(2) / 2.0), 0, 0, sin(rpy(2) / 2.0));

    return (qz * qy * qx).normalized();
}

/* Transform quaternion to RPY angles. */
Eigen::Vector3d quat2RPY(const Eigen::Quaterniond &quat)
{
    // Eigen::Vector3d ypr = quat.matrix().eulerAngles(2, 1, 0);
    
    // return Eigen::Vector3d(ypr(2), ypr(1), ypr(0));
    return Rot2rpy(quat.matrix());
}

/* Transform quaternion to RPY angles. */
Eigen::Vector3d quat2RPY(const Eigen::Vector4d &quat)
{
    return quat2RPY(Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)));
}

/* Transform quaternion vector (w, x, y, z) to rotation matrix. */
Eigen::Matrix3d quat2Rot(const Eigen::Vector4d &quat)
{
    const Eigen::Vector4d qn = quat.normalized();
    const double w = qn(0), x = qn(1), y = qn(2), z = qn(3);

    Eigen::Matrix3d R;
    R(0, 0) = 1.0 - 2.0 * (y * y + z * z);
    R(1, 1) = 1.0 - 2.0 * (x * x + z * z);
    R(2, 2) = 1.0 - 2.0 * (x * x + y * y);
    R(0, 1) = 2.0 * (x * y - w * z);
    R(1, 0) = 2.0 * (x * y + w * z);
    R(0, 2) = 2.0 * (x * z + w * y);
    R(2, 0) = 2.0 * (x * z - w * y);
    R(1, 2) = 2.0 * (y * z - w * x);
    R(2, 1) = 2.0 * (y * z + w * x);

    return R;
}

/* Transform RPY Euler angles to rotation matrix. */
Eigen::Matrix3d rpy2Rot(const Eigen::Vector3d &rpy)
{
    const double r = rpy(0), p = rpy(1), y = rpy(2);

    Eigen::Matrix3d R;
    R(0, 0) = cos(p) * cos(y);
    R(0, 1) = sin(r) * sin(p) * cos(y) - cos(r) * sin(y);
    R(0, 2) = cos(r) * sin(p) * cos(y) + sin(r) * sin(y);
    R(1, 0) = cos(p) * sin(y);
    R(1, 1) = sin(r) * sin(p) * sin(y) + cos(r) * cos(y);
    R(1, 2) = cos(r) * sin(p) * sin(y) - sin(r) * cos(y);
    R(2, 0) = -sin(p);
    R(2, 1) = sin(r) * cos(p);
    R(2, 2) = cos(r) * cos(p);

    return R;
}

/* Transform rotation matrix to RPY Euler angles. */
Eigen::Vector3d Rot2rpy(const Eigen::Matrix3d &rot)
{
    constexpr double eps = std::numeric_limits<double>::epsilon();
    double r, p, y;

    if (std::abs(rot(2, 0) - 1.0) < eps)
    {
        r = 0.0;
        p = -M_PI / 2.0;
        y = std::atan2(-rot(0, 1), -rot(0, 2));
    }
    else if (std::abs(rot(2, 0) + 1.0) < eps)
    {
        r = 0.0;
        p = M_PI / 2.0;
        y = std::atan2(-rot(0, 1), rot(0, 2));
    }
    else
    {
        r = std::atan2(rot(2, 1), rot(2, 2));
        y = std::atan2(rot(1, 0), rot(0, 0));
        if (std::abs(std::cos(y)) > std::abs(std::sin(y)))
        {
            p = std::atan2(-rot(2, 0), rot(0, 0) / std::cos(y));
        }
        else
        {
            p = std::atan2(-rot(2, 0), rot(1, 0) / std::sin(y));
        }
    }

    return Eigen::Vector3d(r, p, y);
}

/* Get skew-symmetric matrix of an R3 vector. */
Eigen::Matrix3d getSkewSymmMat(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d ss = Eigen::Matrix3d::Zero();
    ss(1, 0) = v(2), ss(0, 1) = -ss(1, 0);
    ss(2, 0) = -v(1), ss(0, 2) = -ss(2, 0);
    ss(2, 1) = v(0), ss(1, 2) = -ss(2, 1);

    return ss;
}

/* Calculate angular velocity using given RPY and RPY velocity. */
Eigen::Vector3d rpy2AngVel(const Eigen::Vector3d &rpy, 
                           const Eigen::Vector3d &rpydt)
{
    double avx = rpydt(0) * cos(rpy(1)) - rpydt(2) * cos(rpy(0)) * sin(rpy(1));
    double avy = rpydt(1) + rpydt(2) * sin(rpy(0));
    double avz = rpydt(0) * sin(rpy(1)) + rpydt(2) * cos(rpy(0)) * cos(rpy(1));

    return Eigen::Vector3d(avx, avy, avz);
}

/* Find the point in points with least Euclidean distance 
   to the given point. */
template<unsigned Dim>
typename std::vector<Eigen::Matrix<double, Dim, 1>>::iterator 
closest_it(std::vector<Eigen::Matrix<double, Dim, 1>> &points, 
        const Eigen::Matrix<double, Dim, 1> &point)
{
    double min_dist = (points[0] - point).norm();
    typename std::vector<Eigen::Matrix<double, Dim, 1>>::iterator out = points.begin();

    for (typename std::vector<Eigen::Matrix<double, Dim, 1>>::iterator it = points.begin() + 1; 
         it != points.end(); it++)
    {
        double dist = (*it - point).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            out = it;
        }
    }

    return out;
}

/* Find the point in points with least Euclidean distance 
   to the given point. */
template<unsigned Dim>
Eigen::Matrix<double, Dim, 1> closest(const std::vector<Eigen::Matrix<double, Dim, 1>> &points, 
                                     const Eigen::Matrix<double, Dim, 1> &point)
{
    double min_dist = (points[0] - point).norm();
    Eigen::Matrix<double, Dim, 1> out = points[0];

    for (size_t i = 1; i < points.size(); i++)
    {
        double dist = (points[i] - point).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            out = points[i];
        }
    }

    return out;
}

} // namespace utils
