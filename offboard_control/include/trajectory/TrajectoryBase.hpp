/*
 * filename: TrajectoryBase.hpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Base class of all types of trajectory.
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>

namespace trajectory
{
class TrajectoryBase
{
protected:
    uint8_t _dof;

    enum
    {
        TRAJ_3_DOF = 3, 
        TRAJ_4_DOF = 4,
        TRAJ_6_DOF = 6
    };

public:
    TrajectoryBase(): _dof(6) {}
    TrajectoryBase(uint8_t dof);
    
    virtual ~TrajectoryBase() {}

public:
    virtual bool getTrajPoint(Eigen::VectorXd& traj_pnt, float time_in_sec) = 0;

};

} // namespace trajectory
