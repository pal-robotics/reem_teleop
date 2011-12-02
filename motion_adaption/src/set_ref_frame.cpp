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


#include "motion_adaption/motion_adaption.h"


bool MotionAdaption::setRefFrame()
{
  try
  {
    tf_listener_.waitForTransform(robot_base_str_, robot_ref_torso_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(robot_base_str_, robot_ref_torso_str_, ros::Time(0), tf_robot_ref_torso_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 2) Couldn't get the transformation to the robot's torso.");
    ROS_WARN("No further processing will be done!");    
    return false;
  } 

  tf_ref_frame_.setOrigin(tf_robot_ref_torso_.getOrigin()); // take the position of the robot _reference_ frame
  quat_ = tf::Quaternion(tf_usr_torso_.getRotation()); // and the rotation of the user torso
  // (manually) rotate it to align with the convention (i.e. D-H) of the robot's frames
  quat_adjust_.setRPY(ref_frame_rot_vec_[0], ref_frame_rot_vec_[1], ref_frame_rot_vec_[2]);
  quat_ = quat_adjust_ * quat_;  
  tf_ref_frame_.setRotation(quat_);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_ref_frame_, ros::Time::now(),
  robot_base_str_, "/ref_frame"));  

  // another reference frame without the operator's torso orientation, but with its convention
  quat_.setRPY(ref_frame_rot_vec_[0], ref_frame_rot_vec_[1], ref_frame_rot_vec_[2]);
  tf_ref_frame_.setRotation(quat_);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_ref_frame_, ros::Time::now(),
  robot_base_str_, "/fixed_ref_frame")); 

  return true;
}
