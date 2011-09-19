/*
 * Filename: publish_msgs.cpp
 * Author: Marcus Liebhardt
 * Date: XX.05.2011
 * License: TODO
 */
 
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

