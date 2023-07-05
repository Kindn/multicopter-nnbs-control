#ifndef _DATA_CONVERTER_H_
#define _DATA_CONVERTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <deque>
#include <queue>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ekf_state_estimator/FilePath.h>

#include <eigen3/Eigen/Eigen>

class DataConverter
{
public: 
    DataConverter(const ros::NodeHandle &nh, 
                  const ros::NodeHandle &nh_priv);

    ~DataConverter();

private:
    void est_att_cb(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    void mavros_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    bool save_data_cb(ekf_state_estimator::FilePath::Request &req, 
                      ekf_state_estimator::FilePath::Response &res);

    bool saveData(const std::string &file_path) const;

    static void msgQuat2RPY(const geometry_msgs::Quaternion &quat, 
                     geometry_msgs::Vector3 &rpy);

private:
    ros::NodeHandle nh_, nh_priv_;

    ros::Subscriber est_att_sub_;
    ros::Subscriber mavros_pose_sub_;

    ros::Publisher est_rpy_pub_;
    ros::Publisher mavros_rpy_pub_;

    ros::ServiceServer save_data_srv_;

    int max_vec_size_;
    uint64_t min_record_dt_ns_;

    std::vector<geometry_msgs::Vector3Stamped> est_rpy_vec_;
    std::vector<geometry_msgs::Vector3Stamped> mavros_rpy_vec_;
};

#endif // _DATA_CONVERTER_H_