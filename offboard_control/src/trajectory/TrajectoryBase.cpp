/*
 * filename: TrajectoryBase.cpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Implementation of TrajectoryBase class.
 */

#include "trajectory/TrajectoryBase.hpp"

namespace trajectory
{
TrajectoryBase::TrajectoryBase(uint8_t dof)
{
    if (dof != TrajectoryBase::TRAJ_4_DOF && 
        dof != TrajectoryBase::TRAJ_6_DOF)
        _dof = 6;
    else
        _dof = dof;
}

} // namespace trajectory
