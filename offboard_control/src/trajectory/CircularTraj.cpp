/*
 * filename: CircularTraj.cpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Implementation of CircularTraj class.
 */

#include "trajectory/CircularTraj.hpp"

namespace trajectory
{
CircularTraj::CircularTraj(): TrajectoryBase(6)
{
    _centerx = 0.f;
    _centery = 0.f;
    _z_sp = 2.0;
    _radius = 1.0f;
    _angular_velocity = 0.7f;
    _roll_sp = 0.f;
    _pitch_sp = 0.f;
    _yaw_start = -M_PI;
}

CircularTraj::CircularTraj(float centerx, 
                            float centery, 
                            float z_sp, 
                            float radius, 
                            float angular_velocity, 
                            float yaw_start, 
                            float roll_sp, 
                            float pitch_sp)
{
    _centerx = centerx;
    _centery = centery;
    _z_sp = z_sp;
    _radius = radius;
    _angular_velocity = angular_velocity;
    _roll_sp = roll_sp;
    _pitch_sp = pitch_sp;
    _yaw_start = yaw_start;
}

void CircularTraj::setCenter(float cx, float cy)
{
    _centerx = cx;
    _centery = cy;
}

void CircularTraj::setZ(float z)
{
    assert(z > 0);
    _z_sp = z;
}

void CircularTraj::setRadius(float r)
{
    assert(r > 0);
    _radius = r;
}

void CircularTraj::setAngularVel(float ang_vel)
{
    _angular_velocity = ang_vel;
}

void CircularTraj::setRoll(float roll)
{
    assert(roll > -M_PI / 2 && roll < M_PI / 2);
    _roll_sp = roll;
}

void CircularTraj::setPitch(float pitch)
{
    assert(pitch > -M_PI && pitch < M_PI);
    _pitch_sp = pitch;
}

void CircularTraj::setYawStart(float yaw_start)
{
    assert(yaw_start > -M_PI && yaw_start < M_PI);
    _yaw_start = yaw_start;
}

Eigen::Vector2d CircularTraj::getCenter() const
{
    Eigen::Vector2d center(_centerx, _centery);

    return center;
}

float CircularTraj::getZ() const
{
    return _z_sp;
}

float CircularTraj::getRadius() const
{
    return _radius;
}

float CircularTraj::getAngularVel() const
{
    return _angular_velocity;
}

float CircularTraj::getRoll() const
{
    return _roll_sp;
}

float CircularTraj::getPitch() const
{
    return _pitch_sp;
}

bool CircularTraj::getTrajPoint(Eigen::VectorXd& traj_pnt, float time_in_sec)
{
    if (traj_pnt.size() != _dof)
    {
        Eigen::VectorXd t(_dof);
        traj_pnt = t;
    }

    double phase = _angular_velocity * time_in_sec;
    if (phase >= 2 * M_PI)
        phase -= (2 * M_PI * double(int(phase / (2 * M_PI))));
    double yaw_sp = _yaw_start + phase;
    
    traj_pnt(0) = _centerx + _radius * cos(phase);
    traj_pnt(1) = _centery + _radius * sin(phase);
    traj_pnt(2) = _z_sp;
    traj_pnt(3) = _roll_sp;
    traj_pnt(4) = _pitch_sp;
    traj_pnt(5) = yaw_sp;

    return true;
}

} // namespace trajectory
