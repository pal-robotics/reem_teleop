/*
 * Filename: set_ref_frame.cpp
 * Author: Marcus Liebhardt
 * Date: XX.05.2011
 * License: TODO
 */

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
   
  tf_ref_frame_.setOrigin(tf_robot_ref_torso_.getOrigin());
  quat_ = tf::Quaternion(tf_usr_torso_.getRotation());
  quat_adjust_.setRPY(ref_frame_rot_vec_[0], ref_frame_rot_vec_[1], ref_frame_rot_vec_[2]);
  quat_ = quat_adjust_ * quat_;  
  tf_ref_frame_.setRotation(quat_);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_ref_frame_, ros::Time::now(),
  robot_base_str_, "/ref_frame"));  

  quat_.setRPY(ref_frame_rot_vec_[0], ref_frame_rot_vec_[1], ref_frame_rot_vec_[2]);
  tf_ref_frame_.setRotation(quat_);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_ref_frame_, ros::Time::now(),
  robot_base_str_, "/fixed_ref_frame")); 

  return true;
}
