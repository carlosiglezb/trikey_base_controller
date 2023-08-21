//
// Created by Carlos on 1/31/23.
//

#include "filters.hpp"

ExponentialMovingAverageFilter::ExponentialMovingAverageFilter(
        double dt, double time_constant, Eigen::VectorXd init_value,
        Eigen::VectorXd min_crop, Eigen::VectorXd max_crop) {
  dt_ = dt;
  init_value_ = init_value;
  double T =
          std::max(time_constant, 2 * dt_); // Nyquist-Shannon sampling theorem
  time_constant_ = T;
  alpha_ = 1. - std::exp(-dt_ / T);
  min_crop_ = min_crop;
  max_crop_ = max_crop;

  average_ = init_value_;
}

ExponentialMovingAverageFilter::~ExponentialMovingAverageFilter() {}

void ExponentialMovingAverageFilter::input(Eigen::VectorXd input_value) {
  average_ += alpha_ * (input_value - average_);
  raw_value_ = input_value;

  for (int i = 0; i < average_.size(); ++i) {
    if (average_[i] < min_crop_[i])
      average_[i] = min_crop_[i];
    if (average_[i] > max_crop_[i])
      average_[i] = max_crop_[i];
  }
}

Eigen::VectorXd ExponentialMovingAverageFilter::output() { return average_; }

void ExponentialMovingAverageFilter::clear() { average_ = init_value_; }

void ExponentialMovingAverageFilter::resetTimeConstant(double time_constant)
{
  double T = std::max(time_constant, 2 * dt_); // Nyquist-Shannon sampling theorem
  time_constant_ = T;
  alpha_ = 1. - std::exp(-dt_ / T);
}


FirstOrderLowPassFilter::FirstOrderLowPassFilter(double dt, double period,
                                                 int dim)
    : dt_(dt), dim_(dim) {
  Reset();
  _CutOffPeriod(period);
}
void FirstOrderLowPassFilter::_CutOffPeriod(double period) {
  period = std::max(period, 2 * dt_); // Nyquist-Shannon sampling theorem
  cut_off_period_ = period;
}
void FirstOrderLowPassFilter::Reset() {
  prev_val_ = Eigen::VectorXd::Zero(dim_);
}
void FirstOrderLowPassFilter::Input(const Eigen::VectorXd &new_val) {
  double x = (cut_off_period_ <= dt_) ? 1. : dt_ / cut_off_period_;
  prev_val_ = x * new_val + (1. - x) * prev_val_;
}
Eigen::VectorXd FirstOrderLowPassFilter::Output() { return prev_val_; }
