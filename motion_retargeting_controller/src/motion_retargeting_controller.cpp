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

#include <exception>
#include <ros/ros.h>
#include "motion_retargeting_controller/motion_retargeting.h"

// Globals
std::string node_name = "motion_retargeting_coordinator";

typedef motion_retargeting::MotionRetargeting MoRet;
typedef motion_retargeting::MotionRetargetingConfiguration MoRetConf;

/*
 * Get the parameters from the parameter server and set up the motion retargeting configuration
 */
MoRetConf configureMotionRetargeting()
{
  // general configuration
// retargeting_config.general_config = ...
  // configuration of all retargetings
  unsigned int nr_of_retargetings = 0;
  MoRetConf::GeneralConfigurationParameters general_config_params;
  std::vector<MoRetConf::RetargetingParameters> retargeting_params;

//  for (unsigned int retargeting = 0; retargeting < nr_of_retargetings; ++retargeting)
//  {
//    if (type == "trans_rot_retargeting")
//    {
//      retargeting = new motion_retargeting::TransRotRetargeting(/*parameters*/);
//      if (retargeting)
//      {
//        retargeting_config.retargetings.push_back(retargeting);
//      }
//      else
//      {
//        ROS_ERROR_STREAM("Couldn't create retartegting type 'trans_rot_retargeting' for retargeting '"
//                         << retargeting_name << ". [" << node_name << "]");
//      }
//    }
//  }
  MoRetConf configuration = MoRetConf(general_config_params, retargeting_params);
  return configuration;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ROS_INFO_STREAM("Initialising controller. [" << node_name << "]");
  MoRet mot_ret;
  MoRetConf mo_ret_conf;

  try
  {
    mo_ret_conf = configureMotionRetargeting();
    ROS_INFO_STREAM("Motion retargeting configuration created. [" << node_name << "]");
    mot_ret = MoRet(mo_ret_conf);
    ROS_INFO_STREAM("Motion retargeting ready to rock! [" << node_name << "]");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Caught an exception, while trying to initialise motion retargeting: "
                     << e.what());
    return -1;
  }

  trajectory_msgs::JointTrajectoryPoint output_joint_states;
  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    ros::spinOnce();
    mot_ret.retarget();
    // TODO: Send waypoint to joint controller
    loop_rate.sleep();
  }

  return 0;
}
