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
