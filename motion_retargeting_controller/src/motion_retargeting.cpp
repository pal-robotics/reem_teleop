/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, Yujin Robot
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
 *   * Neither the name of Yujin Robot nor the names of its
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

#include "motion_retargeting_controller/motion_retargeting.h"

namespace motion_retargeting
{

MotionRetargeting::MotionRetargeting(const MotionRetargetingConfiguration& retargeting_config,
                                          const ros::NodeHandle& nh) :
                                          nh_(nh)
{
  motion_adaption_ = motion_adaption::MotionAdaptionPtr(
                     new motion_adaption::MotionAdaption(retargeting_config.getParameters().motion_adaption_parameters));
  tree_kinematics_ = tree_kinematics::TreeKinematicsPtr(
                     new tree_kinematics::TreeKinematics(retargeting_config.getParameters().kinematics_parameters, nh_));
  tree_ik_request_.endpt_names = retargeting_config.getParameters().kinematics_parameters.endpt_names;
  joint_states_subscriber = nh_.subscribe("joint_states", 10, &MotionRetargeting::jointStatesCallback, this);
}

MotionRetargeting::~MotionRetargeting(){};

void MotionRetargeting::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  current_joint_states_ = *joint_states;
  if(!joint_states_initialised_)
  {
    joint_states_initialised_ = true;
    ROS_DEBUG_STREAM("Joint states initialised.");
  }
}

bool MotionRetargeting::retarget(/*trajectory_msgs::JointTrajectoryPoint& output_joint_states*/)
{
  /*
   *  Here I would like to call an object taking care of the motion retargeting _input_ (e.g. openni_tracker tf output)
   *  and outputting convenient data for motion adaption (e.g. to allow motion to use solely an internal transformer)
   */
// tf::TransformStamped input =  input_handler_->getInput()
  /*
   * Motion adaption
   */
  if(!(motion_adaption_->adapt(adapted_entpt_poses_)))
  {
    ROS_WARN_STREAM("Motion adaption failed. Skipping.");
    return false;
  }
  /*
   * Tree kinematics
   */
  if(joint_states_initialised_)
  {
    tree_ik_request_.endpt_poses = adapted_entpt_poses_;
    tree_ik_request_.ik_seed_state = current_joint_states_;
    if (tree_kinematics_->getPositionIk(tree_ik_request_, tree_ik_response_))
    {
      goal_joint_states_ = tree_ik_response_.solution;
    }
    else
    {
      ROS_WARN_STREAM("IK failed. Skipping.");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Can't perform IK, since no joint states have been retrieved yet.");
    return false;
  }
  /*
   *  Here I would like to call an object taking care of the motion retargeting _output_
   *  and outputting convenient data for the target system (e.g. REEM(?), Robosem (FollowJointTrajectoryAction))
   */
//  output_handler_->setOutput(tree_ik_response_)
  return true;
}

} // namespace motion_retargeting
