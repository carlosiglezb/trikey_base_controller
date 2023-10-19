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

#include "trikey_base_controller.h"

namespace trikey_base_controller
{
    TrikeyBaseController::TrikeyBaseController()
    : command_struct_twist_(),
        base_frame_("base_link"),
        odom_frame_("world")
    {
        dt_ = 0.001;
        kp_vel_ = 0.;
        ki_pos_ = 0.;
//        damping_gain_ = 0.;
        vel_filter_tau_ = 0.01;
        kinematics_calculator = new OmniwheelKinematics();

        Eigen::Vector3d zeros = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_limits;
        vel_limits << 1.5, 1.5, 1.5;
        vel_filter_ = new ExponentialMovingAverageFilter(dt_, vel_filter_tau_, zeros, -vel_limits, vel_limits);

        // odom filter 
        odom_filter_dt_ = 1/lidar_frequency_;
        //(double dt, double period, int dim)

        odom_filter_ = new FirstOrderLowPassFilter(odom_filter_dt_, 0.1, 3);

        // initialize karopp compensator
        static_force0_ = 2.15;
        static_force1_ = 1.4575;
        static_force2_ = 1.18;
        viscous_force_ = 0.1;
        vel_deadzone_ = 0.05;


        // Maximum acceleration for wheel velocities
        MAX_ACCEL = 0.01;
    }

    TrikeyBaseController::~TrikeyBaseController()
    {
        delete kinematics_calculator;
        delete vel_filter_;
        // change to smart ptr
        delete odom_filter_;
        delete karnopp_compensator_;
        cmd_sub_.shutdown();
        kp_vel_sub_.shutdown();
//        damping_gain_sub_.shutdown();
        vel_filter_sub_.shutdown();
        ground_truth_sub_.shutdown();
    }

    //Controller initialization (non-real time)
    bool TrikeyBaseController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh)
    {
        filt_vel_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, "/trikey/base_controller/filt_velocities", 5));
        cmd_wheel_vel_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, "/trikey/base_controller/cmd_wheel_velocities", 5));

        // load joints to control
        std::vector<std::string> joint_names;
        if( !nh.getParam( "joint_names", joint_names ) ) {
            ROS_ERROR("Parameter joint_names not specified");
            return false;
        }
        joints_.resize(joint_names.size());
        init_pos_.resize(joint_names.size());
        init_vel_.resize(joint_names.size());
        cmd_wheel_velocities_.setZero();
        filtered_velocities_.setZero();
        raw_velocities_.setZero();
//        noisy_joints_vel_.setZero();
        filt_vel_pub_->msg_.name.resize(joint_names.size());
        filt_vel_pub_->msg_.velocity.resize(joint_names.size());
        cmd_wheel_vel_pub_->msg_.name.resize(joint_names.size());
        cmd_wheel_vel_pub_->msg_.velocity.resize(joint_names.size());

        for(int index = 0; index < joint_names.size(); index++)
        {
            joints_[index] = hw->getHandle(joint_names[index]);
            filt_vel_pub_->msg_.name[index] = joint_names[index];
            filt_vel_pub_->msg_.velocity[index] = 0.0;
            cmd_wheel_vel_pub_->msg_.name[index] = joint_names[index];
            cmd_wheel_vel_pub_->msg_.velocity[index] = 0.0;
            cmd_wheel_velocities_[index] = 0.0;
            filtered_velocities_[index] = 0.0;
            ROS_INFO("added joint %s", joint_names[index].c_str());
        }

        // Set controller parameters
        if( !nh.getParam( "/trikey/base_controller/kp_vel_gain", kp_vel_ ) ) {
          ROS_ERROR("Parameter vel_gain not specified");
          return false;
        }
//        if( !nh.getParam( "/trikey/base_controller/damping_gain", damping_gain_ ) ) {
//          ROS_ERROR("Parameter damping_gain not specified");
//          return false;
//        }
        if( !nh.getParam( "/trikey/base_controller/vel_filter_tau", vel_filter_tau_ ) ) {
          ROS_ERROR("Parameter vel_filter_tau not specified");
          return false;
        }
        vel_filter_->resetTimeConstant(vel_filter_tau_);

        // Set twist to wheel mapping
        double wheel_radius;
        if( !nh.getParam( "/trikey/kinematic_properties/wheel_radius", wheel_radius ) ) {
            ROS_ERROR("Parameter wheel_radius not specified");
            return false;
        }

        // Set twist to wheel to chassis distance
        double distance_wheel_to_chassis;
        if( !nh.getParam( "/trikey/kinematic_properties/wheel_to_chassis_center", distance_wheel_to_chassis ) ) {
            ROS_ERROR("Parameter wheel_radius not specified");
            return false;
        }
        kinematics_calculator->initialize_H(wheel_radius, distance_wheel_to_chassis);
        cmd_sub_ = nh.subscribe("/trikey/base_controller/cmd_vel", 1, &TrikeyBaseController::cmdVelCallback, this);
        kp_vel_sub_ = nh.subscribe("/trikey/base_controller/kp_vel_gain", 1, &TrikeyBaseController::cmdKpVelCallback, this);
        vel_filter_sub_ = nh.subscribe("/trikey/base_controller/vel_filter_tau", 1, &TrikeyBaseController::cmdVelFilterCallback, this);
        //TODO remove ground truth subscriber and compute and publish estimate
        // ground_truth_sub_ = nh.subscribe("/odom", 1, &TrikeyBaseController::odometryCallback, this);
//        filt_vel_pub_.init(nh, "/trikey/base_controller/filt_velocities", 5);

        ROS_INFO("Finished controller initialization");

        if( !nh.getParam( "/trikey/odom_frame", odom_frame_ ) ) {
            ROS_ERROR("Parameter odom_frame not specified");
            return false;
        }
        if( !nh.getParam( "/trikey/base_frame", base_frame_ ) ) {
            ROS_ERROR("Parameter base_frame not specified");
            return false;
        }
        setupOdomPublishers(nh);
        ROS_INFO("Finished odometry initialization");


        return true;
    }

    //Controller startup (real-time)
    void TrikeyBaseController::starting(const ros::Time& time) {
        //Get initial position to use in the control procedure
        for(int j = 0; j < joints_.size(); j++)
        {
            init_pos_[j] = joints_[j].getPosition();
            init_vel_[j] = joints_[j].getVelocity();
            raw_velocities_[j] = joints_[j].getVelocity();
            cmd_wheel_velocities_[j] = 0.0;
            filtered_velocities_[j] = 0.0;
        }
    }

    //Controller running
    void TrikeyBaseController::update(const ros::Time& time, const ros::Duration& period)
    {

      
        // get velocities in vector form for filtering later on
        for(unsigned int j = 0; j < joints_.size(); j++) {
          raw_velocities_[j] = joints_[j].getVelocity();
        }
        vel_filter_->input(raw_velocities_);

        // Read desired twist sent from user/external controller
        CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());

        //compute odometry estimate => Odometry estimated by separate node with Lidar

        // TODO: limit velocities and accelerations

        // Limit velocities <= garbage, fix it later
        // for(unsigned int j = 0; j < joints_.size(); j++) {
        
        //     if (cmd_wheel_velocities_[j] > 0.5) {
        //       cmd_wheel_velocities_[j] = 0.5;
        //     }
        //     else if (cmd_wheel_velocities_[j] < -0.5) {
        //       cmd_wheel_velocities_[j] = -0.5;
        //     }
        // }
        
        // // Limit accelerations
        // for(unsigned int j = 0; j < joints_.size(); j++) {
        //     double delta_vel = cmd_wheel_velocities_[j] - prev_cmd_wheel_velocities_[j];
        //     double dt = period.toSec(); // ROS Duration to seconds
            
        //     // Calculate actual acceleration
        //     double accel = delta_vel / dt;
            
        //     // Limit acceleration
        //     if (std::abs(accel) > MAX_ACCEL) {
        //         accel = std::copysign(MAX_ACCEL, accel); // Keep the sign of acceleration
        //         cmd_wheel_velocities_[j] = prev_cmd_wheel_velocities_[j] + accel * dt;
        //     }
            
        //     // Update previous velocity for the next iteration
        //     prev_cmd_wheel_velocities_[j] = cmd_wheel_velocities_[j];
        // }



    
        

        // compute corresponding desired wheel velocities
        computeWheelVelocities(curr_cmd_twist, cmd_wheel_velocities_);

        auto w0_friction_comp = KarnoppCompensator(static_force0_, viscous_force_, vel_deadzone_);
        auto w1_friction_comp = KarnoppCompensator(static_force1_, viscous_force_, vel_deadzone_);
        auto w2_friction_comp = KarnoppCompensator(static_force2_, viscous_force_, vel_deadzone_);
        // ROS_INFO("friction compensation");
        // Set wheels torques (TODO add torque sensor readings and/or implement velocity control)
        filtered_velocities_ = vel_filter_->output();

  

        for(int j = 0; j < joints_.size(); j++)
        {
        // Feedforawrd term to compensate for friction
          if (j == 0) {
            friction_compensation_ = w0_friction_comp.Update(cmd_wheel_velocities_[j]);
            // ROS_INFO("friction compensation w0: %f", friction_compensation);
          }
          else if (j == 1) {
            friction_compensation_ = w1_friction_comp.Update(cmd_wheel_velocities_[j]);
            // ROS_INFO("friction compensation w1: %f", friction_compensation);
          }
          else if (j == 2) {
            friction_compensation_ = w2_friction_comp.Update(cmd_wheel_velocities_[j]);
            // ROS_INFO("friction compensation w2: %f", friction_compensation);
          }

    


        // Error computation for P controller
          double vel_error = cmd_wheel_velocities_[j] - filtered_velocities_[j];
          // double pos_error = vel_error * dt;

          // double cmd = kp_vel_ * vel_error + friction_compensation_;
          // double cmd = cmd_wheel_velocities_[j];
          double cmd = kp_vel_ * vel_error;
          joints_[j].setCommand(cmd);
        }

        // publish filtered wheel velocities
        if (filt_vel_pub_->trylock()){
          for(unsigned int j = 0; j < joints_.size(); j++) {
            filt_vel_pub_->msg_.velocity[j] = filtered_velocities_[j];
          }
          filt_vel_pub_->msg_.header.stamp = ros::Time::now();
          filt_vel_pub_->unlockAndPublish();
        }
        // publish commanded wheel velocities
        if (cmd_wheel_vel_pub_->trylock()){
          for(unsigned int j = 0; j < joints_.size(); j++) {
            cmd_wheel_vel_pub_->msg_.velocity[j] = cmd_wheel_velocities_[j];
          }
          cmd_wheel_vel_pub_->msg_.header.stamp = ros::Time::now();
          cmd_wheel_vel_pub_->unlockAndPublish();
        }
    }

    //Controller exiting
    void TrikeyBaseController::stopping(const ros::Time& time) { }

    void TrikeyBaseController::cmdKpVelCallback(const std_msgs::Float64 &gain)
    {
      kp_vel_ = std::max(gain.data, 0.);
    }


    void TrikeyBaseController::cmdVelFilterCallback(const std_msgs::Float64 &time_constant)
    {
      vel_filter_tau_ = time_constant.data;
      vel_filter_->resetTimeConstant(vel_filter_tau_);
    }

    void TrikeyBaseController::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
    {
        if(!std::isfinite(cmd_vel.angular.z) || !std::isfinite(cmd_vel.linear.x))
        {
            ROS_WARN_THROTTLE(1.0, "Received NaN in velocity command. Ignoring.");
            return;
        }

            command_struct_twist_.lin_x   = cmd_vel.linear.x;
            command_struct_twist_.lin_y   = cmd_vel.linear.y;
            command_struct_twist_.ang   = cmd_vel.angular.z;
            command_struct_twist_.stamp = ros::Time::now();
            command_twist_.writeFromNonRT(command_struct_twist_);
    }

    void TrikeyBaseController::odometryCallback(const nav_msgs::Odometry& odom)
    {
        ground_truth_ = odom;
    }

    void TrikeyBaseController::computeWheelVelocities(const CommandTwist cmd_twist, Eigen::Vector3d &cmd_wheel_velocities)
    {
        Eigen::Vector3d twist;
        twist(0) = cmd_twist.ang;
        twist(1) = cmd_twist.lin_x;
        twist(2) = cmd_twist.lin_y;
        cmd_wheel_velocities = kinematics_calculator->get_H() * twist;
    }

    void TrikeyBaseController::setupOdomPublishers(ros::NodeHandle& nh)
    {
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "/odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_;
        odom_pub_->msg_.child_frame_id = base_frame_;
        odom_pub_->msg_.pose.pose.position.z = 0.0;
        odom_pub_->msg_.twist.twist.linear.x  = 0.0;
        odom_pub_->msg_.twist.twist.linear.y  = 0.0;
        odom_pub_->msg_.twist.twist.angular.z = 0.0;
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].transform.rotation.w = 1.0;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_;
        tf_odom_pub_->msg_.transforms[0].transform.rotation = ground_truth_.pose.pose.orientation;
    }

    void TrikeyBaseController::updateOdometry(const nav_msgs::Odometry& odometry_)
    {
        // Populate odom message and publish
        // if (odom_pub_->trylock())
        // {
        //     odom_pub_->msg_.header.stamp = ros::Time::now();
        //     odom_pub_->msg_.pose.pose.position.x = odometry_.pose.pose.position.x;
        //     odom_pub_->msg_.pose.pose.position.y = odometry_.pose.pose.position.y;
        //     odom_pub_->msg_.pose.pose.position.z = odometry_.pose.pose.position.z;
        //     odom_pub_->msg_.pose.pose.orientation = odometry_.pose.pose.orientation;
        //     odom_pub_->msg_.twist.twist.linear.x  = odometry_.twist.twist.linear.x;
        //     odom_pub_->msg_.twist.twist.linear.y  = odometry_.twist.twist.linear.y;
        //     odom_pub_->msg_.twist.twist.angular.z = odometry_.twist.twist.angular.z;
        //     odom_pub_->unlockAndPublish();
        // }



        // Publish tf for base w.r.t. world
        if(tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = ros::Time::now();
            odom_frame.header.frame_id = odom_frame_;
            odom_frame.child_frame_id = base_frame_;
            odom_frame.transform.translation.x = odometry_.pose.pose.position.x;
            odom_frame.transform.translation.y = odometry_.pose.pose.position.y;
            odom_frame.transform.translation.z = odometry_.pose.pose.position.z;
            odom_frame.transform.rotation = odometry_.pose.pose.orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

    void TrikeyBaseController::filterOdometry(const nav_msgs::Odometry& odometry_)
    {

        // Filter odometry
        // Divide odometry into pose and twist
        Eigen::Vector3d pose3d_;
        pose3d_ << odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, 0;
        Eigen::Vector3d linVel3d_;
        linVel3d_ << odometry_.twist.twist.linear.x, odometry_.twist.twist.linear.y, 0;
        Eigen::Vector3d angVel3d_;
        angVel3d_ << 0, 0, odometry_.twist.twist.angular.z;

        // Filter pose and twist
        // odom_filter_->Input(pose3d_);
        // Eigen::Vector3d filtered_pose3d_ = odom_filter_->Output();
        // odom_filter_->Input(linVel3d_);
        // Eigen::Vector3d filtered_lin_vel3d_ = odom_filter_->Output();
        // odom_filter_->Input(angVel3d_);
        // Eigen::Vector3d filtered_ang_vel3d_ = odom_filter_->Output();



    } 
}

//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(trikey_base_controller::TrikeyBaseController, controller_interface::ControllerBase);
