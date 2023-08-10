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
    H_mat(0, 0) = wheel_to_chassis_center*cos(2.094)+sin(2.094);
    H_mat(1, 0) = wheel_to_chassis_center*cos(2.094)-0.43311144;
    H_mat(2, 0) = wheel_to_chassis_center*cos(2.094)-0.43311144;

    H_mat(0, 1) = -wheel_to_chassis_center*sin(2.094)+cos(2.094);
    H_mat(1, 1) = -wheel_to_chassis_center*sin(2.094)+0.48595369;
    H_mat(2, 1) = -wheel_to_chassis_center*sin(2.094)+0.48595369;

    H_mat(0,2) = 0;
    H_mat(1,2) = -sqrt(3)/2;
    H_mat(2,2) = -sqrt(3)/2;

    H_mat /= -wheel_radius;

}

Eigen::Matrix3d OmniwheelKinematics::get_H() {return H_mat;}