/*
 * filename: offboard_hover.cpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Offboard control node.
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

#include <eigen3/Eigen/Core>

#include "trajectory/PolynomialTraj.hpp"
#include "trajectory/TrajectoryGenerator.hpp"

using namespace trajectory;

ros::Publisher setpoint_raw_local_pub;

visualization_msgs::Marker points, line_strip;
visualization_msgs::Marker real_traj;

double total_time = 15;

bool ready = false;
bool fly = false;
bool finished = false;

double ready_time;
double fly_time;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

Eigen::Vector3d pos_drone;                     
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, 
                                      msg->pose.position.y, 
                                      msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
    geometry_msgs::Point p;
    p.x = pos_drone(0);
    p.y = pos_drone(1);
    p.z = pos_drone(2);
    real_traj.points.push_back(p);
}

Eigen::Vector3d att_drone;
void att_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat); 
    tf::Matrix3x3(quat).getRPY(att_drone(0), att_drone(1), att_drone(2));
}

Eigen::Vector3d vel_drone;
Eigen::Vector3d ang_vel_drone;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone(0) = msg->twist.linear.x;
    vel_drone(1) = msg->twist.linear.y;
    vel_drone(2) = msg->twist.linear.z;
    ang_vel_drone(0) = msg->twist.angular.x;
    ang_vel_drone(1) = msg->twist.angular.y;
    ang_vel_drone(2) = msg->twist.angular.z;
}

Eigen::Vector3d acc_drone;
Eigen::Vector3d ang_acc_drone;
void acc_cb(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr &msg)
{
    acc_drone(0) = msg->accel.accel.linear.x;
    acc_drone(1) = msg->accel.accel.linear.y;
    acc_drone(2) = msg->accel.accel.linear.z;
    ang_acc_drone(0) = msg->accel.accel.angular.x;
    ang_acc_drone(1) = msg->accel.accel.angular.y;
    ang_acc_drone(2) = msg->accel.accel.angular.z;
}

void sendPoseSetpoint(const Eigen::Vector3d& pos_sp, 
                      const Eigen::Vector3d& vel_sp, 
                      const Eigen::Vector3d& acc_sp, 
                      const Eigen::Vector3d& att_sp, 
                      double t_sec)
{
    mavros_msgs::PositionTarget position_target;
    // mavros_msgs::RollPitchTarget roll_pitch_target;

    position_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_target.type_mask = 0xFFF & (~mavros_msgs::PositionTarget::IGNORE_PX) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_PY) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_PZ) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_YAW) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_VX) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_VY) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_VZ) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_AFX) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_AFY) & 
                                        (~mavros_msgs::PositionTarget::IGNORE_AFZ) & 
                                        (~mavros_msgs::PositionTarget::FORCE);
    position_target.position.x = pos_sp(0);
    position_target.position.y = pos_sp(1);
    position_target.position.z = pos_sp(2);
    position_target.velocity.x = vel_sp(0);
    position_target.velocity.y = vel_sp(1);
    position_target.velocity.z = vel_sp(2);
    position_target.acceleration_or_force.x = acc_sp(0);
    position_target.acceleration_or_force.y = acc_sp(1);
    position_target.acceleration_or_force.z = acc_sp(2);
    position_target.yaw = att_sp(2);

    // ros::Time time_stamp(t_sec);
    // position_target.header.stamp =  roll_pitch_target.header.stamp = time_stamp;

    setpoint_raw_local_pub.publish(position_target);
    // roll_pitch_control_setpoints_pub.publish(roll_pitch_target);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "polynomial_node");
    ros::NodeHandle nh, nh_priv("~");

    std::string output_fpath = "/home/lpy/data.txt";
    nh_priv.param<std::string>("output_fpath", output_fpath, output_fpath);
    std::ofstream ofs;
    ofs.open(output_fpath);
    if (!ofs.is_open())
    {
        ROS_WARN("Failed to open file: %s.", output_fpath.c_str());
    }
    else
    {
        ROS_INFO("Data will be saved to %s.", output_fpath.c_str());
    }

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber local_vel_sub = 
        nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, vel_cb);
    ros::Subscriber acc_sub = 
        nh.subscribe<geometry_msgs::AccelWithCovarianceStamped>("/mavros/local_position/accel", 100, acc_cb);

    ros::Subscriber att_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data", 100, att_cb);

    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    Eigen::Vector3d pos_sp(0.0, 0.0, 2.0);
    Eigen::Vector3d vel_sp(0.0, 0.0, 0.0), acc_sp(0.0, 0.0, 0.0), att_sp(0.0, 0.0, 0.0);
    nh_priv.param<double>("position_target_x", pos_sp(0), pos_sp(0));
    nh_priv.param<double>("position_target_y", pos_sp(1), pos_sp(1));
    nh_priv.param<double>("position_target_z", pos_sp(2), pos_sp(2));

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        sendPoseSetpoint(pos_sp, vel_sp, acc_sp, att_sp, ros::Time::now().toSec());
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time armed_time, fly_time0;
    double t_sec = -1;
    double recording_time = 20;
    nh_priv.param<double>("recording_time", recording_time, recording_time);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed_time = ros::Time::now();
                    // fly = true;
                }
                last_request = ros::Time::now();
            }
        }

        if (!fly)
        {
            sendPoseSetpoint(pos_sp, vel_sp, acc_sp, att_sp, ros::Time::now().toSec());;
            if ((pos_drone - pos_sp).norm() < 0.2)
            {
                fly = true;
                ROS_INFO("Reach target point. Start to record data.");
                fly_time0 = ros::Time::now();
            }
        }
        else
        {
            // t_sec = (ros::Time::now() - armed_time).toSec();
            t_sec = (ros::Time::now() - fly_time0).toSec();
            if (t_sec <= recording_time)
            {
                ofs << t_sec << " " 
                    << pos_sp(0) << " " << pos_sp(1) << " " << pos_sp(2) << " "
                    << pos_drone(0) << " " << pos_drone(1) << " " << pos_drone(2) << " "
                    << vel_sp(0) << " " << vel_sp(1) << " " << vel_sp(2) << " "
                    << vel_drone(0) << " " << vel_drone(1) << " " << vel_drone(2) << " "
                    << acc_sp(0) << " " << acc_sp(1) << " " << acc_sp(2) << " "
                    << acc_drone(0) << " " << acc_drone(1) << " " << acc_drone(2) << std::endl;
            }
            else
            {
               ROS_INFO("Recording finished."); 
            }
        }

        sendPoseSetpoint(pos_sp, vel_sp, acc_sp, att_sp, ros::Time::now().toSec());

        ros::spinOnce();
        rate.sleep();
    }

    ofs.close();

    return 0;
}