//
// Created by Carlos on 11/5/21.
//

#include "omniwheel_kinematics.h"

OmniwheelKinematics::OmniwheelKinematics()
{
    wheel_vel_limit_ = 0.0;

    H_mat.setZero();
}

void OmniwheelKinematics::initialize_H(double wheel_radius, double wheel_to_chassis_center)
{
    H_mat(0, 0) = wheel_to_chassis_center;
    H_mat(1, 0) = wheel_to_chassis_center;
    H_mat(2, 0) = wheel_to_chassis_center;

    H_mat(0, 1) = 1.0;
    H_mat(1, 1) = -0.5;
    H_mat(2, 1) = -0.5;

    H_mat(1,2) = -sin(4.*M_PI/3.0);
    H_mat(2,2) = sin(4.*M_PI/3.0);

    H_mat /= -wheel_radius;
}

Eigen::Matrix3d OmniwheelKinematics::get_H() {return H_mat;}
