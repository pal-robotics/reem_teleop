/*
 * Filename: get_and_align_transforms.cpp
 * Author: Marcus Liebhardt
 * Date: XX.04.2011
 * License: TODO
 */
 
#include "motion_adaption/motion_adaption.h"


bool MotionAdaption::getTransforms()
{
  try
  {
    //torso
    tf_listener_.waitForTransform(world_ref_frame_str_, user_torso_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(world_ref_frame_str_, user_torso_str_, ros::Time(0), tf_usr_torso_);
    //head
    tf_listener_.waitForTransform(user_torso_str_, user_head_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_head_str_, ros::Time(0), tf_usr_head_);
    // right shoulder
    tf_listener_.waitForTransform(user_torso_str_, user_r_shoulder_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_r_shoulder_str_, ros::Time(0), tf_usr_r_shoulder_);
    // right shoulder to right elbow
    tf_listener_.waitForTransform(user_r_shoulder_str_, user_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_r_shoulder_str_, user_r_elbow_str_, ros::Time(0), tf_usr_r_shoulder_elbow_);
    // right elbow to right hand
    tf_listener_.waitForTransform(user_r_elbow_str_, user_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_r_elbow_str_, user_r_hand_str_, ros::Time(0), tf_usr_r_elbow_hand_);
    // right elbow
    tf_listener_.waitForTransform(user_torso_str_, user_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_r_elbow_str_, ros::Time(0), tf_usr_r_elbow_);
    // right hand
    tf_listener_.waitForTransform(user_torso_str_, user_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_r_hand_str_, ros::Time(0), tf_usr_r_hand_);
    // left shoulder
    tf_listener_.waitForTransform(user_torso_str_, user_l_shoulder_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_l_shoulder_str_, ros::Time(0), tf_usr_l_shoulder_);
    // left shoulder to left elbow
    tf_listener_.waitForTransform(user_l_shoulder_str_, user_l_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_l_shoulder_str_, user_l_elbow_str_, ros::Time(0), tf_usr_l_shoulder_elbow_);
    // left elbow to left hand
    tf_listener_.waitForTransform(user_l_elbow_str_, user_l_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_l_elbow_str_, user_l_hand_str_, ros::Time(0), tf_usr_l_elbow_hand_);
    // left elbow
    tf_listener_.waitForTransform(user_torso_str_, user_l_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_l_elbow_str_, ros::Time(0), tf_usr_l_elbow_);
    // left hand
    tf_listener_.waitForTransform(user_torso_str_, user_l_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
    tf_listener_.lookupTransform(user_torso_str_, user_l_hand_str_, ros::Time(0), tf_usr_l_hand_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("(Step 1) Couldn't get one or more transformations of the user.");
    ROS_WARN("No further processing will be done!");
    return false;
  }   
  return true;
}

