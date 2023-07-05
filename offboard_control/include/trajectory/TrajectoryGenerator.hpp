/*
 * filename: TrajectoryGenerator.hpp
 * author:   Peiyan Liu, nROS-LAB, HITSZ
 * E-mail:   1434615509@qq.com
 * brief:    Class of minimum-snap trajectory generator.
 */

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include "PolynomialTraj.hpp"

namespace trajectory
{
#define DEFAULT_TRAJ_DOF             6u
#define DEFAULT_POLY_ORDER           6u
#define DEFAULT_MIN_DER_ORDER        4u

class TrajectoryGenerator
{
private:
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _Ct;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _K;  // K = A^(-1) * M * C
    Eigen::MatrixXd _R;
    Eigen::MatrixXd _Rff;
    Eigen::MatrixXd _Rfp;
    Eigen::MatrixXd _Rpf;
    Eigen::MatrixXd _Rpp;

    std::vector<int> fact;

public:
    TrajectoryGenerator() {}
    ~TrajectoryGenerator() {}

public:
    PolynomialTraj getMinSnapTraj(Eigen::MatrixXd waypoints, 
                                  Eigen::VectorXd start_vel, 
                                  Eigen::VectorXd end_vel, 
                                  Eigen::VectorXd start_acc, 
                                  Eigen::VectorXd end_acc, 
                                  double time_sum_in_sec);
    
    void allocateTimes(std::vector<double>& times, 
                       Eigen::MatrixXd waypoints, 
                       double time_sum_in_sec);
    
    void calcQ(std::vector<double> times);

    void calcA(std::vector<double> times);

    void calcCt(size_t num_seg);

    void calcM(size_t num_seg);

};

inline int factorial(int n)
{
    int result = 1;
    for (int i = n; i > 0; i--)
        result *= i;
    
    return result;
}

} // namespace trajectory
