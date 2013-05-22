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

#include "motion_retargeting/motion_retargeting.h"

namespace motion_retargeting
{

MotionRetargeting::MotionRetargeting(const ros::NodeHandle& nh,
                                          const motion_adaption::MotionAdaptionPtr motion_adaption,
                                          const tree_kinematics::KinematicsParameters& kinematics_params,
                                          const tree_kinematics::TreeKinematicsPtr tree_kinematics,
                                          const MotionRecorderPtr motion_recorder,
                                          const OutputHandlerPtr output_handler) :
                                          nh_(nh),
                                          joint_states_initialised_(false),
                                          process_output_(true),
                                          record_motion_(false)
{
  motion_adaption_ = motion_adaption;
  tree_kinematics_ = tree_kinematics;
  motion_recorder_ = motion_recorder;
  output_handler_ = output_handler;

  tree_ik_request_.endpt_names = kinematics_params.endpt_names;
  joint_states_subscriber_ = nh_.subscribe("joint_states", 10, &MotionRetargeting::jointStatesCallback, this);
  motion_recorder_subscriber_ = nh_.subscribe("record_motion", 10, &MotionRetargeting::motionRecorderCallback, this);
  output_control_subscriber_ = nh_.subscribe("output_control", 10, &MotionRetargeting::outputControlCallback, this);
}

MotionRetargeting::~MotionRetargeting(){};

void MotionRetargeting::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  if(!joint_states_initialised_)
  {
    joint_states_ = *joint_states;
    joint_states_initialised_ = true;
    ROS_DEBUG_STREAM("Joint states initialised.");
  }
  joint_states_ = *joint_states;
//  joint_states_subscriber_.shutdown();
  return;
}

void MotionRetargeting::motionRecorderCallback(const std_msgs::Empty::ConstPtr& msg)
{
  // This is not multi-threading safe! Do not use this callback manually!
  if(record_motion_)
  {
    if (!motion_recorder_->stopRecording())
    {
      ROS_ERROR_STREAM("Motion Retargeting: Error occured while stoping motion recording.");
    }
    record_motion_ = false;
    ROS_INFO_STREAM("Motion Retargeting Controller: Motion recording deactivated.");
  }
  else
  {
    /*
     * TODO: Spin off a separate thread for the recorder in order to avoid slowing down retargeting
     *       (exchange date via shared queues)
     */
    try
    {
      if (motion_recorder_->prepareRecording())
      {
        record_motion_ = true;
        ROS_INFO_STREAM("Motion Retargeting Controller: Motion recording activated.");
      }
      else
      {
        ROS_ERROR_STREAM("Motion Retargeting: Error occured while preparing for motion recording.");
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Motion retargeting: Couldn't initialise the motion recorder!");
      ROS_DEBUG_STREAM(e.what());
    }
  }
  return;
}

void MotionRetargeting::outputControlCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if(process_output_)
  {
    process_output_ = false;
    ROS_INFO_STREAM("Motion Retargeting Controller: Output processing has been deactivated.");
  }
  else
  {
    process_output_ = true;
    ROS_INFO_STREAM("Motion Retargeting Controller: Output processing has been activated.");
  }
  return;
}

bool MotionRetargeting::retarget()
{
  adapted_entpt_poses_.clear();
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
    tree_ik_request_.ik_seed_state = joint_states_;
    if (tree_kinematics_->getPositionIk(tree_ik_request_, tree_ik_response_))
    {
      goal_joint_states_ = tree_ik_response_.solution;
//      joint_states_ = goal_joint_states_;
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
   * Recording
   */
  if(record_motion_)
  {
    if (motion_recorder_)
    {
      motion_recorder_->storeMotion(goal_joint_states_);
    }
    else
    {
      record_motion_ = false;
      ROS_WARN_STREAM("Motion recording was turned on, but a motion recorder hasn't been initialised yet!" <<
                      "Motion recording is now turned off.");
    }
  }
  /*
   * Publish the results
   */
  if (process_output_)
  {
    if (output_handler_)
    {
      if(!(output_handler_->setOutput(goal_joint_states_)))
      {
        ROS_WARN_STREAM("Publishing the goal joint states failed!");
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Can't process output, since output handler hasn't been initialised yet!");
    }
  }
  return true;
}

} // namespace motion_retargeting
