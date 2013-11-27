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

#include <string>
#include <exception>
#include <ros/ros.h>
#include <motion_adaption/motion_adaption.h>
#include <motion_adaption/types/adaption_type.h>
#include <motion_retargeting/motion_retargeting.h>
#include <motion_retargeting/motion_retargeting_parameters.h>
#include <motion_retargeting/collision_checking/collision_checker.h>
#include <motion_retargeting/motion_recorder/rosbagger.h>
#include <motion_retargeting/output_handler/follow_joint_trajectory_action_output_handler.h>
#include <tree_kinematics/tree_kinematics.h>

// Globals
std::string node_name = "robosem_mo_ret_controller";

typedef motion_retargeting::MotionRetargeting MoRet;
typedef motion_retargeting::MotionRetargetingPtr MoRetPtr;


int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh, nh_private("~");
  ROS_INFO_STREAM("Initialising controller. [" << node_name << "]");

  motion_adaption::MotionAdaptionPtr motion_adaption;
  tree_kinematics::TreeKinematicsPtr tree_kinematics;
  motion_retargeting::CollisionCheckerPtr collision_checker;
  motion_retargeting::MotionRecorderPtr motion_recorder;
  motion_retargeting::OutputHandlerPtr output_handler;
  motion_retargeting::MotionRetargetingPtr motion_retargeting;

  double retargeting_frequency = 0.0;
  std::vector<motion_adaption::AdaptionParameters*> motion_adaption_params;
  tree_kinematics::KinematicsParameters kinematics_params;

  if (getMotionRetargetingParameters(nh_private,
                                     retargeting_frequency,
                                     motion_adaption_params,
                                     kinematics_params))
  {
    ROS_INFO_STREAM("Motion retargeting configuration created. [" << node_name << "]");
    try
    {
      // Initialise motion adaption
      motion_adaption = motion_adaption::MotionAdaptionPtr(
                        new motion_adaption::MotionAdaption(motion_adaption_params));
      // Initialise tree kinematics
      tree_kinematics = tree_kinematics::TreeKinematicsPtr(
                        new tree_kinematics::TreeKinematics(kinematics_params, nh));
      // Collision checker not needed, hence not initialised
      // Initialise output handler
      output_handler = motion_retargeting::OutputHandlerPtr(
                       new motion_retargeting::FollowJointTrajectoryActionHandler());
      // Initialise motion recorder
      motion_recorder = motion_retargeting::MotionRecorderPtr(new motion_retargeting::Rosbagger());
      // Initialise motion retargeting
      motion_retargeting = MoRetPtr(new MoRet(nh,
                                              nh_private,
                                              motion_adaption,
                                              kinematics_params,
                                              tree_kinematics,
                                              collision_checker,
                                              output_handler,
                                              motion_recorder));
      ROS_INFO_STREAM("Motion retargeting ready to rock! [" << node_name << "]");
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Caught an exception, while trying to initialise motion retargeting: "
                       << e.what());
      ROS_ERROR_STREAM("Aborting.");
      return -1;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to create the motion retargeting configuration. Aborting.");
    return -1;
  }

  ros::Rate loop_rate(retargeting_frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO_STREAM_THROTTLE(1.0, "Retargeting ... [" << node_name << "]");
    if (!motion_retargeting->retarget())
    {
      ROS_WARN_STREAM("Retargeting failed! [" << node_name << "]");
    }
    loop_rate.sleep();
  }

  std::cout << "Exiting motion retargeting." << std::endl;
  return 0;
}
