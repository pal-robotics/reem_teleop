/*
 * Filename: motion_adaption.h
 * Author: Marcus Liebhardt
 * Date: XX.05.2011
 * License: TODO
 */

#include "motion_adaption/motion_adaption.h"


static const double WAIT_FOR_TF_IN_S = 0.5;
static const std::string NO_NAME_STRING = "no valid transform name";
static const double ROBOT_SHOULDER_HEIGHT_DEFAULT = 0.21732;
static const double ROBOT_SHOULDER_WIDTH_DEFAULT = 0.46382;
static const double ROBOT_SHOULDER_X_OFFSET_DEFAULT = 0.01500;
static const double ROBOT_UPPER_ARM_LENGTH_DEFAULT = 0.23000;
static const double ROBOT_LOWER_ARM_LENGTH_DEFAULT = 0.23950;
static const double ROBOT_HEAD_HEIGHT_DEFAULT = 0.41200;
static const std::string SUB_TOPIC_POSE_TORSO = "pose_torso";
static const std::string SUB_TOPIC_POSE_HEAD = "pose_head";
static const std::string SUB_TOPIC_POSE_ELBOW_RIGHT = "pose_elbow_right";
static const std::string SUB_TOPIC_POSE_HAND_RIGHT = "pose_hand_right";
static const std::string SUB_TOPIC_POSE_ELBOW_LEFT = "pose_elbow_left";
static const std::string SUB_TOPIC_POSE_HAND_LEFT = "pose_hand_left";


MotionAdaption::MotionAdaption(): nh_private_("~")
{
  nh_private_.param("wait_for_tf_in_sec", wait_for_tf_, WAIT_FOR_TF_IN_S);
  nh_private_.param("world_ref_frame_name", world_ref_frame_str_, NO_NAME_STRING);
  nh_private_.param("user/torso_frame_name", user_torso_str_, NO_NAME_STRING);
  nh_private_.param("user/head_frame_name", user_head_str_, NO_NAME_STRING);
  nh_private_.param("user/right_shoulder_frame_name", user_r_shoulder_str_, NO_NAME_STRING);
  nh_private_.param("user/right_elbow_frame_name", user_r_elbow_str_, NO_NAME_STRING);
  nh_private_.param("user/right_hand_frame_name", user_r_hand_str_, NO_NAME_STRING);
  nh_private_.param("user/left_shoulder_frame_name", user_l_shoulder_str_, NO_NAME_STRING);
  nh_private_.param("user/left_elbow_frame_name", user_l_elbow_str_, NO_NAME_STRING);
  nh_private_.param("user/left_hand_frame_name", user_l_hand_str_, NO_NAME_STRING);
  nh_private_.param("robot/shoulder_heigth", robot_shoulder_heigth_, ROBOT_SHOULDER_HEIGHT_DEFAULT);
  nh_private_.param("robot/shoulder_width", robot_shoulder_width_, ROBOT_SHOULDER_WIDTH_DEFAULT);
  nh_private_.param("robot/shoulder_x_offset", robot_shoulder_x_offset_, ROBOT_SHOULDER_X_OFFSET_DEFAULT);
  nh_private_.param("robot/upper_arm_length", robot_upper_arm_length_, ROBOT_UPPER_ARM_LENGTH_DEFAULT);
  nh_private_.param("robot/lower_arm_length", robot_lower_arm_length_, ROBOT_LOWER_ARM_LENGTH_DEFAULT);
  nh_private_.param("robot/head_height", robot_head_height_, ROBOT_HEAD_HEIGHT_DEFAULT);
  robot_arm_length_ = robot_upper_arm_length_ + robot_lower_arm_length_;
  nh_private_.param("robot/base_frame_name", robot_base_str_, NO_NAME_STRING);
  //nh_private_.param("robot/torso_frame_name", robot_torso_str_, NO_NAME_STRING);  
  nh_private_.param("robot/torso_ref_frame_name", robot_ref_torso_str_, NO_NAME_STRING);
  nh_private_.param("robot/head_frame_name", robot_head_str_, NO_NAME_STRING);
  nh_private_.param("robot/right_shoulder_frame_name", robot_r_shoulder_str_, NO_NAME_STRING);      
  nh_private_.param("robot/right_elbow_frame_name", robot_r_elbow_str_, NO_NAME_STRING); 
  nh_private_.param("robot/right_hand_frame_name", robot_r_hand_str_, NO_NAME_STRING); 
  nh_private_.param("robot/left_shoulder_frame_name", robot_l_shoulder_str_, NO_NAME_STRING);      
  //nh_private_.param("robot/left_elbow_frame_name", robot_l_elbow_str_, NO_NAME_STRING); 
  //nh_private_.param("robot/left_hand_frame_name", robot_l_hand_str_, NO_NAME_STRING);    
  nh_private_.param("rotation_angles/ref_frame_rot/r", ref_frame_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/ref_frame_rot/p", ref_frame_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/ref_frame_rot/y", ref_frame_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/torso_goal_rot/r", torso_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/torso_goal_rot/p", torso_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/torso_goal_rot/y", torso_goal_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/head_goal_rot/r", head_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/head_goal_rot/p", head_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/head_goal_rot/y", head_goal_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/right_elbow_goal_rot/r", r_elbow_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/right_elbow_goal_rot/p", r_elbow_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/right_elbow_goal_rot/y", r_elbow_goal_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/right_hand_goal_rot/r", r_hand_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/right_hand_goal_rot/p", r_hand_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/right_hand_goal_rot/y", r_hand_goal_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/left_elbow_goal_rot/r", l_elbow_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/left_elbow_goal_rot/p", l_elbow_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/left_elbow_goal_rot/y", l_elbow_goal_rot_vec_[2], 0.0);
  nh_private_.param("rotation_angles/goals/left_hand_goal_rot/r", l_hand_goal_rot_vec_[0], 0.0);
  nh_private_.param("rotation_angles/goals/left_hand_goal_rot/p", l_hand_goal_rot_vec_[1], 0.0);
  nh_private_.param("rotation_angles/goals/left_hand_goal_rot/y", l_hand_goal_rot_vec_[2], 0.0);  
  pub_torso_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_TORSO, 1);
  pub_head_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_HEAD, 1);
  pub_r_elbow_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_ELBOW_RIGHT, 1);
  pub_r_hand_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_HAND_RIGHT, 1);
  pub_l_elbow_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_ELBOW_LEFT, 1);
  pub_l_hand_pose_ = nh_private_.advertise<geometry_msgs::PoseStamped> (SUB_TOPIC_POSE_HAND_LEFT, 1);

  // initialize variables
  user_shoulder_width_ = user_shoulder_height_ = user_arm_length_ = 0.0;
  x_norm_ = y_norm_ = z_norm_ = 0.0;
  x_adapt_ = y_adapt_ = z_adapt_ = 0.0;
  elbow_x_ = elbow_y_ = 0.0;
  limb_length_ = 0.0;

  tf_ref_frame_.setIdentity();
  tf_torso_goal_.setIdentity();
  tf_head_goal_.setIdentity();
  tf_r_shoulder_elbow_.setIdentity();
  tf_r_shoulder_hand_.setIdentity();
  tf_r_shoulder_scaled_.setIdentity();
  tf_r_shoulder_goal_.setIdentity();
  tf_r_elbow_scaled_.setIdentity();
  tf_r_elbow_pos_.setIdentity();
  tf_l_elbow_orient_.setIdentity();
  tf_r_elbow_hand_.setIdentity();
  tf_r_elbow_goal_.setIdentity();
  tf_r_hand_scaled_.setIdentity();
  tf_r_hand_adjusted_.setIdentity();
  tf_r_hand_goal_.setIdentity();
  tf_l_shoulder_elbow_.setIdentity();
  tf_l_shoulder_hand_.setIdentity();
  tf_l_shoulder_scaled_.setIdentity();
  tf_l_shoulder_goal_.setIdentity();
  tf_l_elbow_scaled_.setIdentity();
  tf_l_elbow_pos_.setIdentity();
  tf_l_elbow_orient_.setIdentity();
  tf_l_elbow_hand_.setIdentity();
  tf_l_elbow_goal_.setIdentity();
  tf_l_hand_scaled_.setIdentity();
  tf_l_hand_goal_.setIdentity();

  tf_usr_r_shoulder_.setIdentity();
  tf_usr_r_shoulder_elbow_.setIdentity();
  tf_usr_r_elbow_hand_.setIdentity();
  tf_usr_r_hand_.setIdentity();
  tf_usr_l_shoulder_.setIdentity();
  tf_usr_l_shoulder_elbow_.setIdentity();
  tf_usr_l_elbow_hand_.setIdentity();
  tf_usr_l_hand_.setIdentity();

  tf_robot_ref_torso_.setIdentity();  
  tf_robot_torso_torso_.setIdentity();
  tf_robot_torso_head_.setIdentity();
  tf_robot_torso_r_shoulder_.setIdentity();
  tf_robot_r_shoulder_l_shoulder_.setIdentity();    
  tf_robot_r_shoulder_r_elbow_.setIdentity();
  tf_robot_r_elbow_r_hand_.setIdentity();  
  
  vec_r_elbow_hand_valid_.setZero();
  vec_l_elbow_hand_valid_.setZero();

  r_elbow_extended_ = false;
  l_elbow_extended_ = false;
}

MotionAdaption::~MotionAdaption()
{
  pub_torso_pose_.shutdown();
  pub_head_pose_.shutdown();
  pub_r_elbow_pose_.shutdown();
  pub_r_hand_pose_.shutdown();
  pub_l_elbow_pose_.shutdown();
  pub_l_hand_pose_.shutdown();
}

void MotionAdaption::adapt()
{
  if(getTransforms())
  {
    if(setRefFrame())
    {
      if(adaptTransforms())
      {
        setGoals();
        //publishData();
      }
    }
  }
}

