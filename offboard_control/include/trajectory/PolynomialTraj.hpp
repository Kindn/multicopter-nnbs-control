/*
 * filename: PolynomialTraj.hpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Class of polynomial trajectory.
 * reference: https://github.com/HKUST-Aerial-Robotics/Fast-Planner
 */

#pragma once

#include "TrajectoryBase.hpp"
#include "utils/math_utils.hpp"
#include <vector>

namespace trajectory
{
class PolynomialTraj : public TrajectoryBase
{
private:
    std::vector<double> _times;            // time of each segment, in second.
    std::vector<std::vector<Eigen::VectorXd>> _coefficients;     // coefficients of polynomials.
                                                                 // shape: [ _dof, _num_seg]

    double _time_sum;   // total time of the trajectory
    size_t _num_seg;    // number of segments

public:
    PolynomialTraj();
    PolynomialTraj(uint8_t dof);
    ~PolynomialTraj() {}

public:
    static PolynomialTraj create();

    static PolynomialTraj create(uint8_t dof);

    void reset(uint8_t dof = 6);

    bool set(std::vector<double> times, 
             std::vector<std::vector<Eigen::VectorXd>> coefficients);

    bool addSegment(std::vector<Eigen::VectorXd> new_seg_coef, 
                    double new_seg_time);

    virtual bool getTrajPoint(Eigen::VectorXd& traj_pnt, float time_in_sec) override;

    bool getTrajPntSeq(std::vector<Eigen::VectorXd>& traj_pnt_seq, double dt_in_sec);

    double getTimeSum() const;

    bool getDerivative(Eigen::VectorXd& derivative, int order, float time_in_sec) const;

    void print() const;
};


} // namespace trajectory
