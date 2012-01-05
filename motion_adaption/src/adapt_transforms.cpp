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


#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btVector3.h>
#include <tf_conversions/tf_eigen.h>
#include "motion_adaption/motion_adaption.h"


bool MotionAdaption::adaptTransforms()
{
  if(adaptTorso())
  {
    if(adaptHead())
    {
      if(scaleUserHandsAndElbows())
      {
        if(adaptShoulders())
        {
          if(adaptElbows())
          {
            if(!adaptHands())
              return false;        
          }        
          else
            return false;
        }
        else
          return false;
      }
      else
        return false;
    }
    else
      return false;
  }
  else
    return false;
      
  return true;
}


bool MotionAdaption::adaptTorso()
{
  // enter code here for setting a torso frame not centered at the robot's reference frame
  
  tf_torso_goal_.setIdentity();
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_torso_goal_, ros::Time::now(), "/ref_frame", "/torso_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_torso_goal_, calc_time, "/ref_frame", "/torso_adapted"));  
  
  return true;
}


bool MotionAdaption::adaptHead()
{
  try
  {
    tf_listener_.waitForTransform("/fixed_ref_frame", robot_head_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform("/fixed_ref_frame", robot_head_str_, ros::Time(0), tf_robot_torso_head_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 3.1) Couldn't get the transformation from the fixed_ref_frame to head_goal_frame.");
    ROS_WARN("No further processing will be done!");
    return false;
  }
  tf_head_goal_.setIdentity();
  tf_head_goal_.setOrigin(tf_robot_torso_head_.getOrigin());
  /*
  quat_ = tf_usr_head_.getRotation();
  quat_adjust_ = tf_robot_torso_head_.getRotation();
  quat_ = quat_ * quat_adjust_;
  tf_head_goal_.setRotation(quat_);
  */
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_head_goal_, ros::Time::now(),"/ref_frame", "/head_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_head_goal_, calc_time,"/ref_frame", "/head_adapted"));
  
  return true;
}


bool MotionAdaption::scaleUserHandsAndElbows()
{ 
  user_shoulder_width_ = tf_usr_r_shoulder_.getOrigin().x() + (- tf_usr_l_shoulder_.getOrigin().x());
  user_shoulder_height_ = (tf_usr_r_shoulder_.getOrigin().y() + tf_usr_l_shoulder_.getOrigin().y()) / 2.0;
  user_upper_arm_length_ = tf_usr_r_shoulder_elbow_.getOrigin().x();
  user_arm_length_ = tf_usr_r_shoulder_elbow_.getOrigin().x() + tf_usr_r_elbow_hand_.getOrigin().x();
  
  if (user_shoulder_width_ > 0.0 && user_shoulder_height_ > 0.0 && user_arm_length_ > 0.0)
  {
    x_norm_ = tf_usr_r_hand_.getOrigin().x() / (user_arm_length_ + 0.5 * user_shoulder_width_);
    y_norm_ = tf_usr_r_hand_.getOrigin().y() / (user_arm_length_ + user_shoulder_height_);
    z_norm_ = tf_usr_r_hand_.getOrigin().z() / user_arm_length_;
    x_adapt_ = x_norm_ * ( robot_arm_length_ + 0.5 * robot_shoulder_width_);
    y_adapt_ = y_norm_ * ( robot_arm_length_ + robot_shoulder_heigth_);
    z_adapt_ = z_norm_ * robot_arm_length_;
    tf_r_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

    x_norm_ = tf_usr_l_hand_.getOrigin().x() / (user_arm_length_ + 0.5 * user_shoulder_width_);
    y_norm_ = tf_usr_l_hand_.getOrigin().y() / (user_arm_length_ + user_shoulder_height_);
    z_norm_ = tf_usr_l_hand_.getOrigin().z() / user_arm_length_;
    x_adapt_ = x_norm_ * ( robot_arm_length_ + 0.5 * robot_shoulder_width_);
    y_adapt_ = y_norm_ * ( robot_arm_length_ + robot_shoulder_heigth_);
    z_adapt_ = z_norm_ * robot_arm_length_;
    tf_l_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

    x_norm_ = tf_usr_r_elbow_.getOrigin().x() / (user_upper_arm_length_ + 0.5 * user_shoulder_width_);
    y_norm_ = tf_usr_r_elbow_.getOrigin().y() / (user_upper_arm_length_ + user_shoulder_height_);
    z_norm_ = tf_usr_r_elbow_.getOrigin().z() / user_upper_arm_length_;
    x_adapt_ = x_norm_ * ( robot_upper_arm_length_ + 0.5 * robot_shoulder_width_);
    y_adapt_ = y_norm_ * ( robot_upper_arm_length_ + robot_shoulder_heigth_);
    z_adapt_ = z_norm_ * robot_upper_arm_length_;
    tf_r_elbow_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

    x_norm_ = tf_usr_l_elbow_.getOrigin().x() / (user_upper_arm_length_ + 0.5 * user_shoulder_width_);
    y_norm_ = tf_usr_l_elbow_.getOrigin().y() / (user_upper_arm_length_ + user_shoulder_height_);
    z_norm_ = tf_usr_l_elbow_.getOrigin().z() / user_upper_arm_length_;
    x_adapt_ = x_norm_ * ( robot_upper_arm_length_ + 0.5 * robot_shoulder_width_);
    y_adapt_ = y_norm_ * ( robot_upper_arm_length_ + robot_shoulder_heigth_);
    z_adapt_ = z_norm_ * robot_upper_arm_length_;
    tf_l_elbow_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));
    
 //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_scaled_, ros::Time::now(), "/ref_frame", "/r_elbow_scaled"));  
    internal_tf.setTransform(tf::StampedTransform(tf_r_elbow_scaled_, calc_time, "/ref_frame", "/r_elbow_scaled"));  
 //   tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_scaled_, ros::Time::now(), "/ref_frame", "/r_hand_scaled"));
    internal_tf.setTransform(tf::StampedTransform(tf_r_hand_scaled_, calc_time, "/ref_frame", "/r_hand_scaled"));
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_scaled_, ros::Time::now(), "/ref_frame", "/l_elbow_scaled"));  
    internal_tf.setTransform(tf::StampedTransform(tf_l_elbow_scaled_, calc_time, "/ref_frame", "/l_elbow_scaled"));  
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_scaled_, ros::Time::now(), "/ref_frame", "/l_hand_scaled"));
    internal_tf.setTransform(tf::StampedTransform(tf_l_hand_scaled_, calc_time, "/ref_frame", "/l_hand_scaled"));
  }
  else
  {
    ROS_WARN("(Step 3.3) User's body proportions are not valid.");
    ROS_WARN("No further processing will be done!");
    return false; 
  }
  return true;
}


bool MotionAdaption::adaptShoulders()
{
  // positions
  try
  {
    tf_listener_.waitForTransform("/fixed_ref_frame", robot_r_shoulder_str_, ros::Time(0),
     ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform("/fixed_ref_frame", robot_r_shoulder_str_, ros::Time(0),
    tf_robot_torso_r_shoulder_);
    tf_listener_.waitForTransform("/fixed_ref_frame", robot_l_shoulder_str_, ros::Time(0),
    ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform("/fixed_ref_frame", robot_l_shoulder_str_, ros::Time(0),
    tf_robot_torso_l_shoulder_);
    tf_listener_.waitForTransform(robot_r_shoulder_str_, robot_l_shoulder_str_, ros::Time(0),
    ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(robot_r_shoulder_str_, robot_l_shoulder_str_, ros::Time(0),
    tf_robot_r_shoulder_l_shoulder_);    
    tf_listener_.waitForTransform(robot_r_shoulder_str_, robot_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(robot_r_shoulder_str_, robot_r_elbow_str_, ros::Time(0), tf_robot_r_shoulder_r_elbow_);
    tf_listener_.waitForTransform(robot_r_elbow_str_, robot_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(robot_r_elbow_str_, robot_r_hand_str_, ros::Time(0), tf_robot_r_elbow_r_hand_);  
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 3.4.1) Couldn't get one or more transformations of the robot to calculate body measurements.");
    ROS_WARN("No further processing will be done!");    
    return false;
  }
  robot_shoulder_heigth_ = sqrt(pow(tf_robot_torso_r_shoulder_.getOrigin()[
  tf_robot_torso_r_shoulder_.getOrigin().absolute().maxAxis()], 2));  
  robot_shoulder_width_ = sqrt(pow(tf_robot_r_shoulder_l_shoulder_.getOrigin()[
  tf_robot_r_shoulder_l_shoulder_.getOrigin().absolute().maxAxis()], 2));  
  robot_upper_arm_length_ = sqrt(tf_robot_r_shoulder_r_elbow_.getOrigin().length2());
  robot_lower_arm_length_ = sqrt(tf_robot_r_elbow_r_hand_.getOrigin().length2());
  robot_arm_length_ = robot_upper_arm_length_ + robot_lower_arm_length_;
  ROS_DEBUG_THROTTLE(1.0, "robot_shoulder_heigth = %f, robot_shoulder_width = %f, robot_arm_length = %f",
  robot_shoulder_heigth_, robot_shoulder_width_, robot_arm_length_);

  tf_r_shoulder_scaled_.setOrigin(tf_robot_torso_r_shoulder_.getOrigin());
  tf_l_shoulder_scaled_.setOrigin(tf_robot_torso_l_shoulder_.getOrigin());
//  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/r_robot_shoulder"));
  internal_tf.setTransform(tf::StampedTransform(tf_r_shoulder_scaled_, calc_time,"/ref_frame", "/r_robot_shoulder"));
// tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/l_robot_shoulder"));
  internal_tf.setTransform(tf::StampedTransform(tf_l_shoulder_scaled_, calc_time,"/ref_frame", "/l_robot_shoulder"));
  /*
  tf_r_shoulder_scaled_.setOrigin(tf::Vector3(robot_shoulder_width_*0;.5, robot_shoulder_heigth_,    
  robot_shoulder_x_offset_));
  tf_l_shoulder_scaled_.setOrigin(tf::Vector3(-robot_shoulder_width_*0.5, robot_shoulder_heigth_, 
  robot_shoulder_x_offset_));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),
  "/ref_frame", "/r_shoulder_scaled"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),
  "/ref_frame", "/l_shoulder_scaled"));
  */
  
  // orientations
  try
  {
    //tf_listener_.waitForTransform("/r_robot_shoulder", "/r_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/r_robot_shoulder", "/r_elbow_scaled", ros::Time(0), tf_r_shoulder_elbow_);
    //tf_listener_.waitForTransform("/r_robot_shoulder", "/r_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/r_robot_shoulder", "/r_hand_scaled", ros::Time(0), tf_r_shoulder_hand_);
    //tf_listener_.waitForTransform("/l_robot_shoulder", "/l_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/l_robot_shoulder", "/l_elbow_scaled", ros::Time(0), tf_l_shoulder_elbow_);
    //tf_listener_.waitForTransform("/l_robot_shoulder", "/l_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/l_robot_shoulder", "/l_hand_scaled", ros::Time(0), tf_l_shoulder_hand_);    
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 3.4.2) Couldn't get one or more transformations from the shoulders to the hands and elbows.");
    ROS_WARN("No further processing will be done!");    
    return false;
  }
  
  // right shoulder 
  vec_shoulder_elbow_ = tf_r_shoulder_elbow_.getOrigin();
  vec_shoulder_hand_ = tf_r_shoulder_hand_.getOrigin();
  if(vec_shoulder_hand_.angle(vec_shoulder_elbow_) < 0.15708 && !vec_r_elbow_hand_valid_.isZero())
  {
    ROS_DEBUG_THROTTLE(0.1, "valid right hand vector is used (angle between vectors: %f).",
    vec_shoulder_hand_.angle(vec_shoulder_elbow_));  
    vec_shoulder_hand_ = vec_shoulder_elbow_ + vec_r_elbow_hand_valid_;
    r_elbow_extended_ = true;
  }
  else
  {
    vec_r_elbow_hand_valid_ = vec_shoulder_hand_ - vec_shoulder_elbow_;
    r_elbow_extended_ = false;
  }
  vec_normal_ = vec_shoulder_hand_.cross(vec_shoulder_elbow_);
  vec_helper_ = vec_normal_.cross(vec_shoulder_hand_);  
  vec_shoulder_hand_.normalize();
  vec_helper_.normalize();
  vec_normal_.normalize();
  mat_orientation_.setValue(vec_shoulder_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                            vec_shoulder_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                            vec_shoulder_hand_.z(), vec_helper_.z(), vec_normal_.z());  
  tf_r_shoulder_goal_.setBasis(mat_orientation_);  
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_goal_, ros::Time::now(), "/r_robot_shoulder", "/r_shoulder_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_r_shoulder_goal_, calc_time, "/r_robot_shoulder", "/r_shoulder_adapted"));
  
  // left shoulder  
  vec_shoulder_elbow_ = tf_l_shoulder_elbow_.getOrigin();
  vec_shoulder_hand_ = tf_l_shoulder_hand_.getOrigin();
  if(vec_shoulder_hand_.angle(vec_shoulder_elbow_) < 0.15708 && !vec_l_elbow_hand_valid_.isZero())
  {
    ROS_DEBUG_THROTTLE(0.1, "valid right hand vector is used (angle between vectors: %f).",
    vec_shoulder_hand_.angle(vec_shoulder_elbow_));  
    vec_shoulder_hand_ = vec_shoulder_elbow_ + vec_l_elbow_hand_valid_;
    l_elbow_extended_ = true;
  }
  else
  {
    vec_l_elbow_hand_valid_ = vec_shoulder_hand_ - vec_shoulder_elbow_;
    l_elbow_extended_ = false;
  }
  vec_normal_ = vec_shoulder_hand_.cross(vec_shoulder_elbow_);
  vec_helper_ = vec_normal_.cross(vec_shoulder_hand_);
  vec_shoulder_hand_.normalize();
  vec_helper_.normalize();
  vec_normal_.normalize();  
  mat_orientation_.setValue(vec_shoulder_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                            vec_shoulder_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                            vec_shoulder_hand_.z(), vec_helper_.z(), vec_normal_.z());  
  tf_l_shoulder_goal_.setBasis(mat_orientation_);
//  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_goal_, ros::Time::now(), "/l_robot_shoulder", "/l_shoulder_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_l_shoulder_goal_, calc_time, "/l_robot_shoulder", "/l_shoulder_adapted"));
  return true;
}


bool MotionAdaption::adaptElbows()
{
  // positions
  limb_length_ = tf_r_shoulder_hand_.getOrigin().length();
  if (limb_length_ >= robot_arm_length_|| r_elbow_extended_)
  {
    elbow_x_ = robot_upper_arm_length_;
    elbow_y_ = 0.0;
  }
  else if (limb_length_ < 1e-6)  
  {
    elbow_x_ = 0.0;
    elbow_y_ = robot_upper_arm_length_;
  }
  else
  {
    elbow_x_ = (pow(limb_length_, 2) - pow(robot_lower_arm_length_, 2) + pow(robot_upper_arm_length_, 2)) / 
    ( 2 * limb_length_);
    elbow_y_ = sqrt(pow(robot_upper_arm_length_, 2) - pow(elbow_x_, 2));     
  }  
  tf_r_elbow_pos_.setOrigin(tf::Vector3(elbow_x_, elbow_y_, 0.0));
  //tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_pos_, ros::Time::now(), "/r_shoulder_adapted", "/r_elbow_pos"));  
  internal_tf.setTransform(tf::StampedTransform(tf_r_elbow_pos_, calc_time, "/r_shoulder_adapted", "/r_elbow_pos"));  
  
  limb_length_ = tf_l_shoulder_hand_.getOrigin().length();
  if (limb_length_ >= robot_arm_length_ || l_elbow_extended_)
  {
    elbow_x_ = robot_upper_arm_length_;
    elbow_y_ = 0.0;
  }
  else if (limb_length_ < 1e-6)  
  {
    elbow_x_ = 0.0;
    elbow_y_ = robot_upper_arm_length_;
  }
  else
  {
    elbow_x_ = (pow(limb_length_, 2) - pow(robot_lower_arm_length_, 2) + pow(robot_upper_arm_length_, 2)) / 
    ( 2 * limb_length_);
    elbow_y_ = sqrt(pow(robot_upper_arm_length_, 2) - pow(elbow_x_, 2));     
  }
  tf_l_elbow_pos_.setOrigin(tf::Vector3(elbow_x_, elbow_y_, 0.0));
//  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_pos_, ros::Time::now(), "/l_shoulder_adapted", "/l_elbow_pos"));  
  internal_tf.setTransform(tf::StampedTransform(tf_l_elbow_pos_, calc_time, "/l_shoulder_adapted", "/l_elbow_pos"));  
  
  // orientations
  try
  {
   // tf_listener_.waitForTransform("/r_elbow_pos", "/r_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/r_elbow_pos", "/r_hand_scaled", ros::Time(0), tf_r_elbow_hand_);
   // tf_listener_.waitForTransform("/l_elbow_pos", "/l_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/l_elbow_pos", "/l_hand_scaled", ros::Time(0), tf_l_elbow_hand_);
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 3.5) Couldn't get one or more transformations from the elbow to the hands.");
    ROS_WARN("No further processing will be done!");    
    return false;
  }
  
  if(l_elbow_extended_ == true)
    tf_l_elbow_hand_.setOrigin(btVector3(robot_lower_arm_length_, 0.0, 0.0));  
  
  vec_elbow_hand_ = tf_r_elbow_hand_.getOrigin();
  vec_normal_ = btVector3(0.0, 0.0, 1.0);  
  vec_helper_ = vec_normal_.cross(vec_elbow_hand_);  
  vec_elbow_hand_.normalize();
  vec_normal_.normalize(); 
  vec_helper_.normalize();  
  mat_orientation_.setValue(vec_elbow_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                            vec_elbow_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                            vec_elbow_hand_.z(), vec_helper_.z(), vec_normal_.z());           
  tf_r_elbow_goal_.setOrigin(tf_r_elbow_pos_.getOrigin());                          
  tf_r_elbow_goal_.setBasis(mat_orientation_);
  
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_goal_, ros::Time::now(), "/r_shoulder_adapted", "/r_elbow_adapted"));    
  internal_tf.setTransform(tf::StampedTransform(tf_r_elbow_goal_, calc_time, "/r_shoulder_adapted", "/r_elbow_adapted"));    
  vec_elbow_hand_ = tf_l_elbow_hand_.getOrigin();
  vec_normal_ = btVector3(0.0, 0.0, 1.0);  
  vec_helper_ = vec_normal_.cross(vec_elbow_hand_);  
  vec_elbow_hand_.normalize();
  vec_normal_.normalize(); 
  vec_helper_.normalize();  
  mat_orientation_.setValue(vec_elbow_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                            vec_elbow_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                            vec_elbow_hand_.z(), vec_helper_.z(), vec_normal_.z());           
  tf_l_elbow_goal_.setOrigin(tf_l_elbow_pos_.getOrigin());                          
  tf_l_elbow_goal_.setBasis(mat_orientation_);
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_goal_, ros::Time::now(), "/l_shoulder_adapted", "/l_elbow_adapted"));  
  internal_tf.setTransform(tf::StampedTransform(tf_l_elbow_goal_, calc_time, "/l_shoulder_adapted", "/l_elbow_adapted"));  
  return true;
}


bool MotionAdaption::adaptHands()
{
  try
  {
   // tf_listener_.waitForTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), tf_r_elbow_orient_);
   // tf_listener_.waitForTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
    internal_tf.lookupTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), tf_l_elbow_orient_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 3.6) Couldn't get one or more transformations from the ref frame to the elbows.");
    ROS_WARN("No further processing will be done!");    
    return false;
  }
  tf_r_hand_adjusted_.setIdentity();
  tf_r_hand_adjusted_.setOrigin(btVector3(robot_lower_arm_length_, 0.0, 0.0));
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_adjusted_, ros::Time::now(), "/r_elbow_adapted", "/r_hand_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_r_hand_adjusted_, calc_time, "/r_elbow_adapted", "/r_hand_adapted"));
  tf_l_hand_goal_.setIdentity();
  tf_l_hand_goal_.setOrigin(btVector3(robot_lower_arm_length_, 0.0, 0.0));
 // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(), "/l_elbow_adapted", "/l_hand_adapted"));
  internal_tf.setTransform(tf::StampedTransform(tf_l_hand_goal_, calc_time, "/l_elbow_adapted", "/l_hand_adapted"));
  return true;
}

