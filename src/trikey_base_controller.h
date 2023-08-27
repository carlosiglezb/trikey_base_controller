/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//
// Created by Carlos Gonzalez on 11/1/21.
//

#ifndef SRC_TRIKEY_BASE_CONTROLLER_H
#define SRC_TRIKEY_BASE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "omniwheel_kinematics.h"

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>

#include "filters.hpp"
#include <random>   //TODO remove

#include "karnopp_compensator.hpp"

#include <memory>


namespace trikey_base_controller
{

    class TrikeyBaseController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        TrikeyBaseController();
        ~TrikeyBaseController();
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh);
        void update(const ros::Time& time, const ros::Duration& period);
        void starting(const ros::Time& time);
        void stopping(const ros::Time& time);

    private:
        struct CommandTwist
        {
            double lin_x;
            double lin_y;
            double ang;

            ros::Time stamp;
            CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
        };

        void cmdVelCallback(const geometry_msgs::Twist &cmd_vel);
        void cmdKpVelCallback(const std_msgs::Float64 &gain);
        void cmdVelFilterCallback(const std_msgs::Float64 &gain);
        void computeWheelVelocities(const CommandTwist cmd_twist,
                                    Eigen::Vector3d &cmd_wheel_velocities);
        void setupOdomPublishers(ros::NodeHandle &nh);
        void odometryCallback(const nav_msgs::Odometry& odom);
        void updateOdometry(const nav_msgs::Odometry& odom);
        
         
        

    private:
        std::vector<hardware_interface::JointHandle> joints_;
        double dt_;
        double kp_vel_;
        double ki_pos_;
        double vel_filter_tau_;
        ExponentialMovingAverageFilter *vel_filter_;
//        Eigen::Vector3d noisy_joints_vel_;  // TODO remove, for testing purposes only
//        std::default_random_engine generator;   // TODO remove, for testing purposes only
//        std::normal_distribution<double> dist;  // TODO remove, for testing purposes only

        std::vector<double> init_pos_;
        std::vector<double> init_vel_;

        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> filt_vel_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> cmd_wheel_vel_pub_;

        ros::Subscriber cmd_sub_;
        ros::Subscriber kp_vel_sub_;
        ros::Subscriber vel_filter_sub_;

        realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
        CommandTwist command_struct_twist_;
        Eigen::Vector3d cmd_wheel_velocities_;
        Eigen::Vector3d filtered_velocities_;
        Eigen::Vector3d raw_velocities_;

        /**
         * Odometry
         */
        ros::Subscriber ground_truth_sub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

        /// Frame to use for odometry and odom tf:
        std::string odom_frame_;

        /// Frame to use for the robot base:
        std::string base_frame_;

        nav_msgs::Odometry ground_truth_;

    
      
        FirstOrderLowPassFilter *odom_filter_;
        double odom_filter_dt_;
        float lidar_frequency_;
        void filterOdometry(const nav_msgs::Odometry& odometry_);

        /**
         * Friction compensator
         */
        double static_force0_;
        double static_force1_;
        double static_force2_;
        double viscous_force_;
        double vel_deadzone_;
        KarnoppCompensator *karnopp_compensator_;

    public:
        OmniwheelKinematics *kinematics_calculator;
    };
}

#endif //SRC_TRIKEY_BASE_CONTROLLER_H
