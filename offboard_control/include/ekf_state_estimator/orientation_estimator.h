/*
 * filename: PolynomialTraj.h
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Definition of OrientationEstimator class.
 */

#ifndef _ORIENTATION_ESTIMATOR_H_
#define _ORIENTATION_ESTIMATOR_H_

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>

#include <eigen3/Eigen/Eigen>

class OrientationEstimator
{
public:
    OrientationEstimator(const ros::NodeHandle &nh, 
                         const ros::NodeHandle &nh_priv);
    
    ~OrientationEstimator();

private:
    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);

    void magn_cb(const sensor_msgs::MagneticField::ConstPtr &msg);

    bool lookUpMagnData(const ros::Time &time_stamp, 
                        sensor_msgs::MagneticField &magn_data); 

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::Publisher estimated_orientation_pub_;

    ros::Subscriber imu_sub_;
    ros::Subscriber magn_sub_;

    Eigen::Matrix4d P_; // a posteriori error covariance
    Eigen::Matrix4d Q_;
    Eigen::Matrix3d R1_, R2_;
    Eigen::Matrix<double, 3, 4> H1_, H2_;
    Eigen::Vector4d q_; // a posteriori orientation

    std::deque<sensor_msgs::MagneticField> magn_data_queue_;
    sensor_msgs::MagneticField latest_magn_data_;

    uint64_t timestamp_tolerence_ns_;

    ros::Time last_time_stamp_;
};

#endif // _ORIENTATION_ESTIMATOR_H_
