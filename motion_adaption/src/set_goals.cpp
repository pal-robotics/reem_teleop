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


void MotionAdaption::setGoals()
{
  tf::StampedTransform out_tf;
  // torso goal
  tf_torso_goal_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  //quat_.setRPY(M_PI, M_PI/2, 0);
  quat_.setRPY(torso_goal_rot_vec_[0], torso_goal_rot_vec_[1], torso_goal_rot_vec_[2]);
  tf_torso_goal_.setRotation(quat_);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_torso_goal_, ros::Time::now(), "/ref_frame", "/torso_goal"));
  
  // head goal
  tf_head_goal_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  //quat_.setRPY(M_PI, M_PI/2, 0);
  quat_.setRPY(head_goal_rot_vec_[0], head_goal_rot_vec_[1], head_goal_rot_vec_[2]);
  tf_head_goal_.setRotation(quat_);
  internal_tf.setTransform(tf::StampedTransform(tf_head_goal_, ros::Time::now(), "/head_adapted", "/head_goal"));
  internal_tf.lookupTransform("/ref_frame", "/head_goal", ros::Time(0), out_tf);
  ////// TEST REMAPING TF
  tf_broadcaster_.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),  "/ref_frame", "/head_goal"));
  
  // right elbow goal
  tf_r_elbow_goal_.setOrigin(btVector3(0.0, 0.0, 0.0));
  //quat_.setRPY(M_PI, 0.0, M_PI);
  quat_.setRPY(r_elbow_goal_rot_vec_[0], r_elbow_goal_rot_vec_[1], r_elbow_goal_rot_vec_[2]);
  tf_r_elbow_goal_.setRotation(quat_);
  internal_tf.setTransform(tf::StampedTransform(tf_r_elbow_goal_, ros::Time::now(), "/r_elbow_adapted", "/r_elbow_goal"));

  internal_tf.lookupTransform("/ref_frame", "/r_elbow_goal", ros::Time(0), out_tf);
  ////// TEST REMAPING TF
  tf_broadcaster_.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),  "/ref_frame", "/r_elbow_goal"));


  // left elbow goal  
  tf_l_elbow_goal_.setOrigin(btVector3(0.0, 0.0, 0.0));
  //quat_.setRPY(0.0, M_PI, 0.0);
  quat_.setRPY(l_elbow_goal_rot_vec_[0], l_elbow_goal_rot_vec_[1], l_elbow_goal_rot_vec_[2]);
  tf_l_elbow_goal_.setRotation(quat_);
  internal_tf.setTransform(tf::StampedTransform(tf_l_elbow_goal_, ros::Time::now(), "/l_elbow_adapted", "/l_elbow_goal"));
  
  internal_tf.lookupTransform("/ref_frame", "/l_elbow_goal", ros::Time(0), out_tf);
  ////// TEST REMAPING TF
  tf_broadcaster_.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),  "/ref_frame", "/l_elbow_goal"));
  
  // right hand goal
  tf_r_hand_goal_.setOrigin(btVector3(0.0, 0.0, 0.0));
  //quat_.setRPY(0.0, M_PI/2, 0.0);
  quat_.setRPY(r_hand_goal_rot_vec_[0], r_hand_goal_rot_vec_[1], r_hand_goal_rot_vec_[2]);
  tf_r_hand_goal_.setRotation(quat_);
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_goal_, ros::Time::now(), "/r_hand_adapted", "/r_hand_goal"));
  internal_tf.setTransform(tf::StampedTransform(tf_r_hand_goal_, ros::Time::now(),  "/r_hand_adapted", "/r_hand_goal"));

  internal_tf.lookupTransform("/ref_frame", "/r_hand_goal", ros::Time(0), out_tf);
  ////// TEST REMAPING TF
  tf_broadcaster_.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),  "/ref_frame", "/r_hand_goal"));

  // left hand goal
  tf_l_hand_goal_.setOrigin(btVector3(0.0, 0.0, 0.0));
  //quat_.setRPY(0.0, -M_PI/2, 0.0);
  quat_.setRPY(l_hand_goal_rot_vec_[0], l_hand_goal_rot_vec_[1], l_hand_goal_rot_vec_[2]);
  tf_l_hand_goal_.setRotation(quat_);
  //tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(),  "/l_hand_adapted", "/l_hand_goal"));
  internal_tf.setTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(),  "/l_hand_adapted", "/l_hand_goal"));
  

  internal_tf.lookupTransform("/ref_frame", "/l_hand_goal", ros::Time(0), out_tf);
  ////// TEST REMAPING TF
  tf_broadcaster_.sendTransform(tf::StampedTransform(out_tf, ros::Time::now(),  "/ref_frame", "/l_hand_goal"));
  /* un-comment this part to published unscaled but aligned transforms
  
  /* un-comment this part to published unscaled but aligned transforms
  
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_torso_aligned_, ros::Time::now(),
  "/ref_frame", "/torso_aligned"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_head_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_head"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_r_shoulder_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_r_shoulder"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_r_elbow_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_r_elbow"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_r_hand_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_r_hand"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_l_shoulder_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_l_shoulder"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_l_elbow_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_l_elbow"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_usr_l_hand_, ros::Time::now(),
  "/torso_aligned", "/tf_usr_l_hand"));
  */
}
