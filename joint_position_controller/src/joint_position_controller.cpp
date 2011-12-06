/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/** \author Marcus Liebhardt */



#include <pluginlib/class_list_macros.h>
#include "joint_position_controller/joint_position_controller.h"


static const std::string SUB_TOPIC_JOINT_STATES_CMD = "joint_states_cmd";


void JointPositionController::jointStatesCmdCB(const sensor_msgs::JointState::ConstPtr& command)
{
  if (command->position.size() == q_desi_.rows())
  {
    for(unsigned int i = 0; i < q_desi_.rows(); ++i)
    {
      q_desi_(i)  = command->position[i];
    }
  }
  else
    ROS_WARN("Size of the commanded joint state does not match the internally used size of the joint array!");
    ROS_WARN("Desired joint positions are not updated!");
}


// Controller initialization in non-realtime
bool JointPositionController::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& nh)
{
  // Store the robot handle for later use
  robot_state_ = robot;

  // Initialize tree from robot model
  tree_.init(robot_state_);

  // Resize (pre-allocate) the variables in non-realtime.
  nrOfJnts_ = tree_.size();
  q_.resize(nrOfJnts_);
  q_desi_.resize(nrOfJnts_);
  q_err_.resize(nrOfJnts_);
  q_err_old_.resize(nrOfJnts_);
  q_err_dot_.resize(nrOfJnts_);
  tau_.resize(nrOfJnts_);
  tau_max_.resize(nrOfJnts_);
  Kp_.resize(nrOfJnts_);
  Kd_.resize(nrOfJnts_);

  // flag for whether or not to limit error
  nh.param("limit_error", limit_error_, false);
  ROS_INFO("limit_error = %s", (limit_error_)?"true":"false");

  // Maximum error
  double factor;
  nh.param("q_err_max_factor", factor, 0.0);
  nh.param("q_err_max", q_err_max_, 0.0);
  ROS_INFO("q_err_max_ = %f", q_err_max_);

  // Maximum error velocity
  nh.param("q_err_dot_max_factor", factor, 0.0);
  q_err_dot_max_ = factor * q_err_max_;
  ROS_INFO("q_err_dot_max_ = %f", q_err_dot_max_);

  // Pick the gains
  double PGain, DGain, DGainFactor;
  nh.param("PGain", PGain, 0.0);
  ROS_INFO("PGain = %f", PGain);
  nh.param("DGainFactor", DGainFactor, 0.0);
  nh.param("DGain", DGain, 0.0);
  ROS_INFO("DGain = %f", DGain);
  for (unsigned int i = 0 ; i < nrOfJnts_ ; ++i)
  {
    Kp_(i) = 0.5 * PGain;
    Kd_(i) = 0.5 * DGain;
  }
  for (unsigned int i = 0 ; i < nrOfJnts_; ++i)
  {
    std::string name_str;
    std::stringstream ss;
    ss << "Kp_" << i;
    ss >> name_str;
    double factor;
    nh.getParam(name_str, factor);
    Kp_(i) = factor * PGain;
    ROS_INFO("Kp_(%d) = %f", i, Kp_(i));
    ss << "Kd_" << i;
    ss >> name_str;
    nh.getParam(name_str, factor);
    Kd_(i) = factor * DGain;
    ROS_INFO("Kd_(%d) = %f", i, Kd_(i));
  }

  double tau_max_value;
  nh.param("tau_max_value", tau_max_value, 100.0);
  for (unsigned int i = 0 ; i < nrOfJnts_; ++i)
  {
    std::string name_str;
    std::stringstream ss;
    ss << "tau_max_factor_" << i;
    ss >> name_str;
    double factor;
    nh.param(name_str, factor, 0.0);
    tau_max_(i) = factor * tau_max_value;
    ROS_INFO("tau_max_(%d) = %f", i, tau_max_(i));
  }

  // Subscribe to the topic for joint states command
  sub_joint_states_cmd = nh.subscribe(SUB_TOPIC_JOINT_STATES_CMD, 20, &JointPositionController::jointStatesCmdCB, this);

  return true;
}


// Controller startup in real-time
void JointPositionController::starting()
{
  loop_count_ = 0;
  last_time_  = robot_state_->getTime();
  dt_         = 0.0;
  for (unsigned int i = 0; i < nrOfJnts_; ++i)
  {
    q_(i) = q_desi_(i) = q_err_(i) = q_err_old_(i) = q_err_dot_(i) = tau_(i) = 0.0;
  }
}


// Controller update loop in real-time
void JointPositionController::update()
{
  if (loop_count_ % 1000 == 0)
  {
    ROS_DEBUG("-----------------------------------UPDATING_START--------------------------------------");
    ROS_DEBUG("loop_count_ = %d", loop_count_);
  }

  // Current joint positions
  tree_.getPositions(q_);

  // Calculate the dt between cycles
  dt_ = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();
  if (loop_count_ % 1000 == 0)
    ROS_DEBUG("dt_ = %f", dt_);

  // Current error
  KDL::Subtract(q_desi_, q_, q_err_);

  // check, if one (or more) joint velocities exceed the maximum value
  // and if so, safe the biggest overshoot for scaling q_dot_ properly
  // to keep the direction of the velocity vector the same
  double rel_os, rel_os_max = 0.0; // relative overshoot and the biggest relative overshoot
  bool max_exceeded = false;
  if(limit_error_ == true)
  {
    ROS_DEBUG_THROTTLE(1.0, "limitting");
    for (unsigned int i = 0; i < q_err_.rows(); ++i)
    {
      if ( q_err_(i) > q_err_max_ )
      {
        max_exceeded = true;
        rel_os = (q_err_(i) - q_err_max_) / q_err_max_;
        if ( rel_os > rel_os_max )
        {
          rel_os_max = rel_os;
        }
      }
      else if ( q_err_(i) < -q_err_max_ )
      {
        max_exceeded = true;
        rel_os = (-q_err_(i) - q_err_max_) / q_err_max_;
        if ( rel_os > rel_os_max)
        {
          rel_os_max = rel_os;
        }
      }
    }
    // scales q_out, if one or more joints exceed their maximum value
    if ( max_exceeded == true )
    {
      KDL::Multiply(q_err_, ( 1.0 / ( 1.0 + rel_os_max ) ), q_err_);
    }
  }

  // calculating error velocity = the change of the error
  KDL::Subtract(q_err_, q_err_old_, q_err_dot_);

  // save the current error for next cycle
  q_err_old_ = q_err_;

  // calculate fake efforts
  for (unsigned int i = 0; i < nrOfJnts_; ++i)
  {
    tau_(i) = Kp_(i) * q_err_(i) + Kd_(i) * q_err_dot_(i);
  }

  // Limit efforts
  for (unsigned int i = 0; i < nrOfJnts_; ++i)
  {
    if (tau_(i) > tau_max_(i))
      tau_(i) = tau_max_(i);
    else if (tau_(i) < -tau_max_(i))
      tau_(i) = -tau_max_(i);
  }

  // And finally send these efforts out
  tree_.setEfforts(tau_);

  if (loop_count_ % 1000 == 0)
  {
    ROS_DEBUG("tree joint controller statistics:");
    for (unsigned int i = 0; i < q_.rows(); ++i)
    {
      ROS_DEBUG("q_[%d]: %f, q_desi_[%d]: %f, q_err_[%d]: %f, q_err_dot_[%d]: %f, tau_[%d]: %f",
      i, q_(i), i, q_desi_(i), i, q_err_(i), i, q_err_dot_(i), i, tau_(i));
    }
    ROS_DEBUG("-----------------------------------UPDATING_END--------------------------------------");
  }

  loop_count_++;
}


// Controller stopping in real-time
void JointPositionController::stopping()
{}


// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(joint_position_controller, JointPositionController, JointPositionController, pr2_controller_interface::Controller)
