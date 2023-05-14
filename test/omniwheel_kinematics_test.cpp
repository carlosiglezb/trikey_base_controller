//
// Created by Carlos on 11/24/21.
//

#include "gtest/gtest.h"

#include "../src/omniwheel_kinematics.h"

namespace base_controller{

static double tolerance = 1.0e-6;

class OmniwheelKinematicsTest : public ::testing::Test
{
protected:
    OmniwheelKinematicsTest()
    {
        baseKinematics = OmniwheelKinematics();
        baseKinematics.initialize_H(0.2, 0.25);
    }
    OmniwheelKinematics baseKinematics;
};

    TEST_F(OmniwheelKinematicsTest, moveUp)
    {
        Eigen::Vector3d moveUp, desiredWheelVelocities;
        moveUp << 0.0, 0.0, 1.0;    // (wz, vx, vy)
//        desiredWheelVelocities << 0.0, 1.0, -1.0;
        desiredWheelVelocities << 0.0, -1.0, 1.0;
        desiredWheelVelocities.normalize();

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * moveUp;
        commandedWheelVelocities.normalize();

        ASSERT_LE((desiredWheelVelocities - commandedWheelVelocities).norm(), tolerance);
    }

    TEST_F(OmniwheelKinematicsTest, moveDown)
    {
        Eigen::Vector3d moveDown, desiredWheelVelocities;
        moveDown << 0.0, 0.0, -1.0;    // (wz, vx, vy)
//        desiredWheelVelocities << 0.0, -1.0, 1.0;
        desiredWheelVelocities << 0.0, 1.0, -1.0;
        desiredWheelVelocities.normalize();

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * moveDown;
        commandedWheelVelocities.normalize();

        ASSERT_LE((desiredWheelVelocities - commandedWheelVelocities).norm(), tolerance);
    }


    TEST_F(OmniwheelKinematicsTest, moveRight)
    {
        Eigen::Vector3d moveRight, desiredWheelVelocities;
        moveRight << 0.0, 1.0, 0.0;     // (wz, vx, vy)

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * moveRight;
        commandedWheelVelocities.normalize();

        Eigen::Vector3d velocityOutput = H.inverse() * commandedWheelVelocities;
        velocityOutput.normalize();

        ASSERT_LE((moveRight - velocityOutput).norm(), tolerance);
    }

    TEST_F(OmniwheelKinematicsTest, moveLeft)
    {
        Eigen::Vector3d moveLeft, desiredWheelVelocities;
        moveLeft << 0.0, -1.0, 0.0;     // (wz, vx, vy)

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * moveLeft;
        commandedWheelVelocities.normalize();

        Eigen::Vector3d velocityOutput = H.inverse() * commandedWheelVelocities;
        velocityOutput.normalize();

        ASSERT_LE((moveLeft - velocityOutput).norm(), tolerance);
    }

    TEST_F(OmniwheelKinematicsTest, rotateClockwise)
    {
        Eigen::Vector3d rotateClockwise, desiredWheelVelocities;
        rotateClockwise << 1.0, 0.0, 0.0;     // (wz, vx, vy)
        desiredWheelVelocities << -1.0, -1.0, -1.0;
        desiredWheelVelocities.normalize();

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * rotateClockwise;
        commandedWheelVelocities.normalize();

        ASSERT_LE((desiredWheelVelocities - commandedWheelVelocities).norm(), tolerance);
    }

    TEST_F(OmniwheelKinematicsTest, rotateCounterClockwise)
    {
        Eigen::Vector3d rotateCounterClockwise, desiredWheelVelocities;
        rotateCounterClockwise << -1.0, 0.0, 0.0;     // (wz, vx, vy)
        desiredWheelVelocities << 1.0, 1.0, 1.0;
        desiredWheelVelocities.normalize();

        Eigen::Matrix3d H = baseKinematics.get_H();
        Eigen::Vector3d commandedWheelVelocities = H * rotateCounterClockwise;
        commandedWheelVelocities.normalize();

        ASSERT_LE((desiredWheelVelocities - commandedWheelVelocities).norm(), tolerance);
    }

}   // namespace base_controller


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}