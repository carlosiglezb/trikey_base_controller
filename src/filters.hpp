//
// Created by Carlos on 1/31/23.
//

#ifndef SRC_FILTERS_HPP
#define SRC_FILTERS_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>

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

class FirstOrderLowPassFilter {
public:
  FirstOrderLowPassFilter(double dt, double period, int dim);

  void Reset();

  void Input(const Eigen::VectorXd &new_val);

  Eigen::VectorXd Output();

private:
  void _CutOffPeriod(double period);

  Eigen::VectorXd prev_val_;
  double cut_off_period_ = 0.;
  double dt_ = 0.005;
  int dim_ = 3;
};

class EncoderConverter {
  public:
    EncoderConverter(int tpr, double gearRatio = 1.0);

    double ticksToRadians(int encoderTicks) const;

  private:
    int TPR_;
    double gearRatio_;

};

#endif //SRC_FILTERS_HPP
