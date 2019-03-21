/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::Publisher pub_;
ros::Publisher bias_pub_;

bool twist_is_zero_;
bool odom_is_zero_;

ros::Time twist_last_move_;
ros::Time odom_last_move_;
ros::Duration twist_standstill_delay_;
ros::Duration odom_standstill_delay_;

double cmd_vel_threshold_;
double odom_threshold_;

// Implement an exponentially weighted moving average to calculate bias
double accumulator_alpha_;
geometry_msgs::Vector3 angular_velocity_accumulator;

// Returns true if |val1| < val2
bool abslt(const double& val1, const double& val2){
  return std::abs(val1) < val2;
}

double accumulator_update(const double& alpha, const double& avg, const double& meas){
  return alpha * meas + (1.0 - alpha) * avg;
}

void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg){
  if(abslt(msg->linear.x, cmd_vel_threshold_) &&
     abslt(msg->linear.y, cmd_vel_threshold_) &&
     abslt(msg->linear.z, cmd_vel_threshold_) &&
     abslt(msg->angular.x, cmd_vel_threshold_) &&
     abslt(msg->angular.y, cmd_vel_threshold_) &&
     abslt(msg->angular.z, cmd_vel_threshold_)){
    twist_is_zero_ = true;
    return;
  }
  else {
    twist_last_move_ = ros::Time::now();
  }
  twist_is_zero_ = false;
}

void odom_callback(const nav_msgs::OdometryConstPtr& msg){
  if(abslt(msg->twist.twist.linear.x, odom_threshold_) &&
     abslt(msg->twist.twist.linear.y, odom_threshold_) &&
     abslt(msg->twist.twist.linear.z, odom_threshold_) &&
     abslt(msg->twist.twist.angular.x, odom_threshold_) &&
     abslt(msg->twist.twist.angular.y, odom_threshold_) &&
     abslt(msg->twist.twist.angular.z, odom_threshold_)){
    odom_is_zero_ = true;
    return;
  }
  else {
    odom_last_move_ = ros::Time::now();
  }
  odom_is_zero_ = false;
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
  sensor_msgs::ImuPtr imu(new sensor_msgs::Imu(*msg));

  const ros::Time n = ros::Time::now();
  const bool is_twist_standing_still = (n - twist_last_move_) > twist_standstill_delay_;
  const bool is_odom_standing_still = (n - odom_last_move_) > odom_standstill_delay_;
  
  if(is_twist_standing_still && is_odom_standing_still){ // Update bias, set outputs to 0
    angular_velocity_accumulator.x = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.x, msg->angular_velocity.x);
    angular_velocity_accumulator.y = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.y, msg->angular_velocity.y);
    angular_velocity_accumulator.z = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.z, msg->angular_velocity.z);
    imu->angular_velocity.x = 0.0;
    imu->angular_velocity.y = 0.0;
    imu->angular_velocity.z = 0.0;

    // Publish bias information
    geometry_msgs::Vector3StampedPtr bias(new geometry_msgs::Vector3Stamped());
    bias->header = imu->header;
    bias->vector = angular_velocity_accumulator;
    bias_pub_.publish(bias);
  } else { // Modify outputs by bias
    imu->angular_velocity.x -= angular_velocity_accumulator.x;
    imu->angular_velocity.y -= angular_velocity_accumulator.y;
    imu->angular_velocity.z -= angular_velocity_accumulator.z;
  }

  // Publish transformed message
	pub_.publish(imu);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "imu_bias_remover");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  // Initialize
  twist_is_zero_ = false;
  odom_is_zero_ = false;
  angular_velocity_accumulator.x = 0.0;
  angular_velocity_accumulator.y = 0.0;
  angular_velocity_accumulator.z = 0.0;

  // Get parameters
  pnh.param<double>("accumulator_alpha", accumulator_alpha_, 0.01);
  pnh.param<double>("cmd_vel_threshold", cmd_vel_threshold_, 0.001);
  pnh.param<double>("odom_threshold", odom_threshold_, 0.001);
  
  twist_standstill_delay_.fromSec(pnh.param<double>("cmd_vel_delay", 2.0));
  odom_standstill_delay_.fromSec(pnh.param<double>("odom_delay", 2.0));

  bool use_cmd_vel;
  pnh.param<bool>("use_cmd_vel", use_cmd_vel, false);
  bool use_odom;
  pnh.param<bool>("use_odom", use_odom, false);
  
  if (!use_cmd_vel && !use_odom){
    ROS_WARN("Both use_cmd_vel and use_odom are disabled, no bias will be removed");
  }
  
  ros::Subscriber cmd_sub;
  if(use_cmd_vel){
    twist_last_move_ = ros::TIME_MAX;
    cmd_sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);
  } else {
    twist_last_move_ = ros::TIME_MIN;
  }
  ros::Subscriber odom_sub;
  if(use_odom){
    odom_last_move_ = ros::TIME_MAX;
    odom_sub = n.subscribe("odom", 1, odom_callback);
  } else {
    odom_last_move_ = ros::TIME_MIN;
  }

  // Create publisher
  pub_ = n.advertise<sensor_msgs::Imu>("imu_biased", 10);
  bias_pub_ = n.advertise<geometry_msgs::Vector3Stamped>("bias", 10, 1);

  // Imu Subscriber
  ros::Subscriber sub = n.subscribe("imu", 100, imu_callback);  
  
  ros::spin();

  return 0;
}
