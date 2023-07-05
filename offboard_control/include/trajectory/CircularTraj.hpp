/*
 * filename: CircularTraj.hpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Class of circular trajectory.
 */

#pragma once

#include "TrajectoryBase.hpp"

namespace trajectory
{
class CircularTraj: public TrajectoryBase
{
private:
    float _centerx, _centery;  // in meter
    float _z_sp;              // in meter
    float _radius;            // in meter
    float _angular_velocity;  // in rad/s

    float _roll_sp, _pitch_sp; // in rad
    float _yaw_start;          // in rad

public:
    CircularTraj();
    CircularTraj(float centerx, 
                 float centery,  
                 float z_sp, 
                 float radius = 1.0f, 
                 float angular_velocity = 0.7f, 
                 float yaw_start = -M_PI, 
                 float roll_sp = 0.f, 
                 float pitch_sp = 0.f);

    static CircularTraj create()
    {
        return CircularTraj();
    }
    static CircularTraj create(float centerx, 
                        float centery,  
                        float z_sp, 
                        float radius = 1.0f, 
                        float angular_velocity = 0.7f, 
                        float yaw_start = -M_PI, 
                        float roll_sp = 0.f, 
                        float pitch_sp = 0.f)
    {
        return CircularTraj(centerx, 
                            centery, 
                            z_sp, 
                            radius, 
                            angular_velocity, 
                            yaw_start, 
                            roll_sp, 
                            pitch_sp);
    }
    
    ~CircularTraj() {}

public:
    void setCenter(float cx, float cy);
    void setZ(float z);
    void setRadius(float r);
    void setAngularVel(float ang_vel);
    void setRoll(float roll);
    void setPitch(float pitch);
    void setYawStart(float yaw_start);

    Eigen::Vector2d getCenter() const;
    float getZ() const;
    float getRadius() const;
    float getAngularVel() const;
    float getRoll() const;
    float getPitch() const;

    virtual bool getTrajPoint(Eigen::VectorXd& traj_pnt, float time_in_sec) override;

};

} // namespace trajectory

