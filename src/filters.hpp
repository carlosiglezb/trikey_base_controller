//
// Created by Carlos on 1/31/23.
//

#ifndef SRC_FILTERS_HPP
#define SRC_FILTERS_HPP

#include <Eigen/Dense>

/// class ExponentialMovingAverageFilter
// https://github.com/stephane-caron/lipm_walking_controller/blob/29b3583e3be91ed6336df25434b6baea1fc9f650/include/lipm_walking/utils/ExponentialMovingAverage.h
class ExponentialMovingAverageFilter {
public:
    ExponentialMovingAverageFilter(double dt, double time_constant,
                                   Eigen::VectorXd init_value,
                                   Eigen::VectorXd min_crop,
                                   Eigen::VectorXd max_crop);
    ~ExponentialMovingAverageFilter();

    void input(Eigen::VectorXd input_value);
    Eigen::VectorXd output();
    void clear();
    void resetTimeConstant(double time_constant);

private:
    double dt_;
    double time_constant_;
    double alpha_;
    Eigen::VectorXd init_value_;
    Eigen::VectorXd average_;
    Eigen::VectorXd raw_value_;
    Eigen::VectorXd min_crop_;
    Eigen::VectorXd max_crop_;
};

#endif //SRC_FILTERS_HPP
