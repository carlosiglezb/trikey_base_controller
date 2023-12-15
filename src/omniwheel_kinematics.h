//
// Created by Carlos on 11/5/21.
//

#ifndef SRC_OMNIWHEEL_KINEMATICS_H
#define SRC_OMNIWHEEL_KINEMATICS_H

#include <eigen3/Eigen/Dense>

class OmniwheelKinematics
{
public:
    OmniwheelKinematics();

    void initialize_H(double wheel_radius, double wheel_to_chassis_center);
    Eigen::Matrix3d get_H(void);
    Eigen::Matrix3d get_H_pinv(void);

private:
    double wheel_vel_limit_;

    Eigen::Matrix3d H_mat;

    Eigen::Matrix3d H_pinv_mat;

};

#endif //SRC_OMNIWHEEL_KINEMATICS_H
