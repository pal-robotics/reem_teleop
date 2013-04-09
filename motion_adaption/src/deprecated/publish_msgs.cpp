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


void MotionAdaption::publishData()
{
  // Torso
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/torso_adapted";
  pose_.pose.position.x = tf_torso_goal_.getOrigin().x();
  pose_.pose.position.y = tf_torso_goal_.getOrigin().y();
  pose_.pose.position.z = tf_torso_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_torso_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_torso_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_torso_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_torso_goal_.getRotation().w();
  pub_torso_pose_.publish(pose_);

  // Head
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/head_adapted";
  pose_.pose.position.x = tf_head_goal_.getOrigin().x();
  pose_.pose.position.y = tf_head_goal_.getOrigin().y();
  pose_.pose.position.z = tf_head_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_head_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_head_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_head_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_head_goal_.getRotation().w();
  pub_head_pose_.publish(pose_);

  // Right elbow
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/r_elbow_adapted";
  pose_.pose.position.x = tf_r_elbow_goal_.getOrigin().x();
  pose_.pose.position.y = tf_r_elbow_goal_.getOrigin().y();
  pose_.pose.position.z = tf_r_elbow_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_r_elbow_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_r_elbow_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_r_elbow_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_r_elbow_goal_.getRotation().w();
  pub_r_elbow_pose_.publish(pose_);

  // Right hand
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/r_hand_adapted";
  pose_.pose.position.x = tf_r_hand_goal_.getOrigin().x();
  pose_.pose.position.y = tf_r_hand_goal_.getOrigin().y();
  pose_.pose.position.z = tf_r_hand_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_r_hand_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_r_hand_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_r_hand_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_r_hand_goal_.getRotation().w();
  pub_r_hand_pose_.publish(pose_);

  // Left elbow
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/l_elbow_adapted";
  pose_.pose.position.x = tf_l_elbow_goal_.getOrigin().x();
  pose_.pose.position.y = tf_l_elbow_goal_.getOrigin().y();
  pose_.pose.position.z = tf_l_elbow_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_l_elbow_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_l_elbow_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_l_elbow_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_l_elbow_goal_.getRotation().w();
  pub_l_elbow_pose_.publish(pose_);

  // Left hand
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = "/l_hand_adapted";
  pose_.pose.position.x = tf_l_hand_goal_.getOrigin().x();
  pose_.pose.position.y = tf_l_hand_goal_.getOrigin().y();
  pose_.pose.position.z = tf_l_hand_goal_.getOrigin().z();
  pose_.pose.orientation.x = tf_l_hand_goal_.getRotation().x();
  pose_.pose.orientation.y = tf_l_hand_goal_.getRotation().y();
  pose_.pose.orientation.z = tf_l_hand_goal_.getRotation().z();
  pose_.pose.orientation.w = tf_l_hand_goal_.getRotation().w();
  pub_l_hand_pose_.publish(pose_);
}

