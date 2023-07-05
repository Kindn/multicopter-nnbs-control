/*
 * filename: PolynomialTraj.cpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Implementation of PolynomialTraj class.
 */

#include "trajectory/PolynomialTraj.hpp"
#include <iostream>

namespace trajectory
{
PolynomialTraj::PolynomialTraj(): TrajectoryBase(6)
{
    _times.reserve(6);
    _coefficients.reserve(6);
    _time_sum = 0.0f;
    _num_seg = 0;
}

PolynomialTraj::PolynomialTraj(uint8_t dof): TrajectoryBase(dof)
{
    // assert((dof == TrajectoryBase::TRAJ_4_DOF) || \
    //        (dof == TrajectoryBase::TRAJ_6_DOF) || \ 
    //        (dof == TrajectoryBase::TRAJ_3_DOF));

    _times.reserve(dof);
    _coefficients.reserve(dof);
    _time_sum = 0.0f;
    _num_seg = 0;
}

PolynomialTraj 
PolynomialTraj::create()
{
    return PolynomialTraj();
}

PolynomialTraj 
PolynomialTraj::create(uint8_t dof)
{
    return PolynomialTraj(dof);
}

void 
PolynomialTraj::reset(uint8_t dof)
{
    _times.clear(), _coefficients.clear();
    _times.reserve(dof), _coefficients.reserve(dof);
    _time_sum = 0.0f;
    _num_seg = 0;
}

bool 
PolynomialTraj::set(std::vector<double> times, 
                    std::vector<std::vector<Eigen::VectorXd>> coefficients)
{
    size_t num_seg = times.size();
    uint8_t dof = coefficients.size();

    for (size_t i = 0; i < dof; i++)
        if (coefficients[i].size() != num_seg)
            return false;
    _time_sum = 0,0;
    for (size_t i = 0; i < num_seg; i++)
        _time_sum += times[i];
    
    _times = times;
    _coefficients = coefficients;
    _num_seg = num_seg;
    _dof = dof;

    return true;
}

bool 
PolynomialTraj::addSegment(std::vector<Eigen::VectorXd> new_seg_coef, 
                           double new_seg_time)
{
    if (new_seg_coef.size() != _dof)
        return false;

    for (int i = 0; i < _dof; i++)
    {
        _coefficients[i].push_back(new_seg_coef[i]);
    }
        
    _times.push_back(new_seg_time);
    _num_seg++;

    return true;
}

bool 
PolynomialTraj::getTrajPoint(Eigen::VectorXd& traj_pnt, 
                             float time_in_sec)
{
    if (time_in_sec < 0)
        time_in_sec = 0;
    else if (time_in_sec > _time_sum)
        time_in_sec = _time_sum;
    
    /*find which segment is in*/
    int idx = 0;
    while(_times[idx] + 1e-4 < time_in_sec)
    {
        time_in_sec -= _times[idx];
        idx++;
        if (idx >= _num_seg - 1)
            break;
    }

    traj_pnt = Eigen::VectorXd::Zero(_dof);
    for (int i = 0; i < _dof; i++)
    {
        int order = _coefficients[0][idx].size() - 1;
        if (order < 0)
        {
            std::cerr << "[Error]";
            std::cerr << "In PolynomialTraj::getTrajPoint: ";
            std::cerr << "Polynomial order should be >= 0!" << std::endl;
            return false;
        }
        else
        {
            Eigen::VectorXd tv(order + 1);
            for (int j = 0; j < tv.size(); j++)
                tv(j) = pow((double)time_in_sec, (double)j);
            traj_pnt(i) = _coefficients[i][idx].dot(tv);
        }
    }

    return true;
}

bool 
PolynomialTraj::getDerivative(Eigen::VectorXd& derivative, int order, float time_in_sec) const
{
    if (time_in_sec < 0)
        time_in_sec = 0;
    else if (time_in_sec > _time_sum)
        time_in_sec = _time_sum;
    
    /*find which segment is in*/
    int idx = 0;
    while(_times[idx] + 1e-9 < time_in_sec)
    {
        time_in_sec -= _times[idx];
        idx++;
        if (idx >= _num_seg - 1)
            break;
    }

    derivative = Eigen::VectorXd::Zero(_dof);
    for (int i = 0; i < _dof; i++)
    {
        int degree = _coefficients[0][idx].size() - 1;
        if (degree < 0)
        {
            std::cerr << "[Error]";
            std::cerr << "In PolynomialTraj::getTrajPoint: ";
            std::cerr << "Polynomial order should be >= 0!";
            return false;
        }
        else
        {
            Eigen::VectorXd tv(degree + 1);
            tv.setZero();
            for (int j = order; j < tv.size(); j++)
                tv(j) = math_utils::factorial<double>(j) / math_utils::factorial<double>(j - order) 
                * pow((double)time_in_sec, (double)(j - order));
            derivative(i) = _coefficients[i][idx].dot(tv);
        }
    }

    return true;
}

bool 
PolynomialTraj::getTrajPntSeq(std::vector<Eigen::VectorXd>& traj_pnt_seq, 
                              double dt_in_sec)
{
    if (_num_seg == 0)
    {
        traj_pnt_seq.clear();
        return false;
    }
    else
    {
        double t = 0.0;
        traj_pnt_seq.clear();
        while (t <= _time_sum)
        {
            Eigen::VectorXd traj_pnt;
            if (getTrajPoint(traj_pnt, t))
            {
                traj_pnt_seq.push_back(traj_pnt);
                t += dt_in_sec;
            }        
            else
            {
                traj_pnt_seq.clear();
                return false;
            }
        }

        return true;
    }
}

double 
PolynomialTraj::getTimeSum() const
{
    return _time_sum;
}

void 
PolynomialTraj::print() const
{
    for (size_t seg = 0; seg < _num_seg; seg++)
    {
        std::cout << "Segment " << seg << ",";
        std::cout << "time: " << _times[seg] << " second(s)";
        for (size_t dof = 0; dof < _dof; dof++)
        {
            std::cout << std::endl;
            std::cout << "\t";
            std::cout << "[" << _coefficients[dof][seg].transpose() << "]";
        }
        std::cout << std::endl;
    }
}

} // namespace trajectory
