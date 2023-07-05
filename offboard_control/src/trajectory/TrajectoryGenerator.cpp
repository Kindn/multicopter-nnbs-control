/*
 * filename: TrajectoryGenerator.cpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Implementation of TrajectoryGenerator class.
 */

#include "trajectory/TrajectoryGenerator.hpp"

namespace trajectory
{
PolynomialTraj 
TrajectoryGenerator::getMinSnapTraj(Eigen::MatrixXd waypoints, 
                                    Eigen::VectorXd start_vel, 
                                    Eigen::VectorXd end_vel, 
                                    Eigen::VectorXd start_acc, 
                                    Eigen::VectorXd end_acc, 
                                    double time_sum_in_sec)
{
    size_t dof = waypoints.rows();
    assert((dof == waypoints.rows()) && \
           (dof == start_vel.size()) && \
           (dof == end_vel.size()) && \
           (dof == start_acc.size()) && \
           (dof == end_acc.size()));
    
    size_t num_seg = waypoints.cols() - 1;
    uint8_t order = 5;

    std::vector<double> times;
    allocateTimes(times, waypoints, time_sum_in_sec);

    fact.clear();
    for (int i = 0; i < order + 1; i++)
        fact.push_back(factorial(i));

    size_t num_f, num_p;
    num_f = num_seg + 5;
    num_p = 2 * (num_seg - 1);

    calcQ(times);
    calcA(times);
    calcCt(num_seg);
    calcM(num_seg);
    Eigen::MatrixXd C = _Ct.transpose();
    _K = _A.inverse() * _M * C;
    _R = _K.transpose() * _Q * _K;
    _Rff = _R.block(0, 0, num_f, num_f);
    _Rfp = _R.block(0, num_f, num_f, num_p);
    _Rpf = _R.block(num_f, 0, num_p, num_f);
    _Rpp = _R.block(num_f, num_f, num_p, num_p);

    std::vector<std::vector<Eigen::VectorXd>> coefficients;
    for (size_t i = 0; i < dof; i++)
    {
        Eigen::VectorXd df(num_f);
        
        df(0) = waypoints(i, 0);
        df(1) = start_vel(i);
        df(2) = start_acc(i);
        df(num_f - 3) = waypoints(i, num_seg);
        df(num_f - 2) = end_vel(i);
        df(num_f - 1) = end_acc(i);
        for(int k = 1; k < num_seg; k++)
            df(k + 2) = waypoints(i, k);

        /*close form optimal dp*/
        Eigen::VectorXd dp = -_Rpp.inverse() * _Rpf * df;
        
        /*get coefficients*/
        Eigen::VectorXd d(num_f + num_p);
        d.segment(0, num_f) = df;
        d.segment(num_f, num_p) = dp;
        Eigen::VectorXd p = _K * d;
        std::vector<Eigen::VectorXd> coef_ith_dof;
        for (int k = 0; k < num_seg; k++)
            coef_ith_dof.push_back(p.segment(k * (order + 1), order + 1));
        coefficients.push_back(coef_ith_dof);
    }

    PolynomialTraj traj(dof);
    traj.set(times, coefficients);

    return traj;
}

void 
TrajectoryGenerator::allocateTimes(std::vector<double>& times, 
                                   Eigen::MatrixXd waypoints, 
                                   double time_sum_in_sec)
{
    std::vector<double> seg_len;
    double len_sum = 0.0;

    for (size_t i = 0; i < waypoints.cols() - 1; i++)
    {
        double len = (waypoints.col(i) - waypoints.col(i + 1)).norm();
        seg_len.push_back(len);
        len_sum += len;
    }

    times.clear();
    for (size_t i = 0; i < waypoints.cols() - 1; i++)
        times.push_back(time_sum_in_sec * seg_len[i] / len_sum);
    
}

void 
TrajectoryGenerator::calcQ(std::vector<double> times)
{
    uint8_t order = 5;
    size_t seg_num = times.size();

    size_t Q_size = (order + 1) * seg_num;
    _Q = Eigen::MatrixXd::Zero(Q_size, Q_size);
    
    uint8_t min_order = DEFAULT_MIN_DER_ORDER;
    for (size_t k = 0; k < seg_num; k++)
    {
        Eigen::MatrixXd Qk; 
        Qk = Eigen::MatrixXd::Zero(order + 1, order + 1);
        for (size_t i = min_order; i <= order; i++)
        {
            for (size_t j = min_order; j <= order; j++)
            {
                int n = i + j - 2 * min_order + 1;
                Qk(i, j) = (double)((fact[i] * fact[j]) / 
                            (fact[i - min_order] * fact[j - min_order] * n)) * 
                            pow(times[k], (double)n);
            }
        }
        size_t start_row = k * (order + 1);
        size_t start_col = k * (order + 1);
        _Q.block(start_row, start_col, order + 1, order + 1) = Qk;
    }
}

void 
TrajectoryGenerator::calcA(std::vector<double> times)
{
    uint8_t order = 5;
    size_t seg_num = times.size();

    size_t A_rows = 6 * seg_num;
    size_t A_cols = (order + 1) * seg_num;
    _A = Eigen::MatrixXd::Zero(A_rows, A_cols);

    for (size_t k = 0; k < seg_num; k++)
    {
        Eigen::MatrixXd Ak = Eigen::MatrixXd::Zero(6, order + 1);
        for (size_t s = 0; s < 3; s++)
        {
            Ak(s, s) = fact[s];
            for (size_t i = s; i < order + 1; i++)
            {
                Ak(s + 3, i) = (double)(fact[i] / fact[i - s]) * 
                                pow(times[k], (double)(i - s));
            }
            _A.block(6 * k, (order + 1) * k, 6, order + 1) = Ak;
        }
    }
}

void 
TrajectoryGenerator::calcCt(size_t num_seg)
{
    size_t Ct_size = 3 * (num_seg + 1);
    _Ct = Eigen::MatrixXd::Zero(Ct_size, Ct_size);
    
    size_t num_f, num_p;
    num_f = num_seg + 5;
    num_p = 2 * (num_seg - 1);

    for (size_t i = 0; i < 3; i++)
    {
        _Ct(i, i) = 1;
        _Ct(num_f - 3 + i, 3 * num_seg + i) = 1;
    }
    for (size_t i = 3; i < num_f - 3; i++)
        _Ct(i, 3 * (i - 2)) = 1;
    for (size_t i = num_f - 3; i < num_f; i++)
        _Ct(i, 2 * num_seg + i - 2) = 1;
    for (size_t i = 0; i < num_seg - 1; i++)
    {
        _Ct(num_f + 2 * i, 4 + 3 * i) = 1;
        _Ct(num_f + 2 * i + 1, 5 + 3 * i) = 1;
    }
}

void 
TrajectoryGenerator::calcM(size_t num_seg)
{
    size_t M_rows = 6 * num_seg;
    size_t M_cols = 3 * (num_seg + 1);
    _M = Eigen::MatrixXd::Zero(M_rows, M_cols);

    for (size_t i = 0; i < num_seg; i++)
        for (size_t j = 0; j < 6; j++)
            _M(6 * i + j, 3 * i + j) = 1;
}

} // namespace trajectory
