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
#include <motion_adaption/types/adaption_type.h>
#include "motion_retargeting_controller/motion_retargeting.h"

// Globals
std::string node_name = "motion_retargeting_controller";

//typedef motion_retargeting::MotionRetargetingParameters MoRetParams;
typedef motion_retargeting::MotionRetargeting MoRet;
typedef motion_retargeting::MotionRetargetingPtr MoRetPtr;
typedef motion_retargeting::MotionRetargetingConfiguration MoRetConf;
typedef motion_retargeting::MotionRetargetingConfigurationPtr MoRetConfPtr;

/*
 * Get the parameters from the parameter server and set up the motion retargeting configuration
 * TODO: Move this into the motion retargeting or some other "tools" class?
 */
bool createMotionRetargetingConfiguration(const ros::NodeHandle&  nh_private,
                                               MoRetConfPtr mo_ret_conf)
{
  motion_retargeting::GeneralParameters mo_ret_general_params;
  /*
   * general configuration
   */
//  if(!nh_private.getParam("general_configuration/retargeting_freq", mo_ret_params.general_parameters.retargeting_freq))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'general_configuration/retargeting_freq' from parameter server!");
//    return false;
//  }
  if(!nh_private.getParam("general_configuration/retargeting_freq", mo_ret_general_params.retargeting_freq))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'general_configuration/retargeting_freq' from parameter server!");
    return false;
  }
  /*
   * motion adaption configuration
   */
  std::vector<motion_adaption::AdaptionParameters> mo_adapt_params;
  motion_adaption::GeneralParameters mo_adapt_general_params;
  if(!nh_private.getParam("motion_adaption/adaption_name", mo_adapt_general_params.adaption_name))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/adaption_name' from parameter server!");
    return false;
  }
  motion_adaption::TransRotParameters trans_rot_params;
  motion_adaption::TransRotAdaptionParameters trans_rot_adapt_params(mo_adapt_general_params, trans_rot_params);
  mo_adapt_params.push_back(trans_rot_adapt_params);
//  motion_adaption::TransRotAdaptionParameters mo_adapt_params;
//  if(!nh_private.getParam("motion_adaption/adaption_name", mo_adapt_params.adaption_name))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/adaption_name' from parameter server!");
//    return false;
//  }
//  std::string adaption_type;
//  if(nh_private.getParam("motion_adaption/adaption_type", adaption_type))
//  {
//    if (adaption_type == "trans_rot_adaption")
//    {
//      mo_adapt_params.adaption_type = motion_adaption::MotionAdaptionParameters::TransRotAdaption;
//    }
//    else
//    {
//      ROS_ERROR_STREAM("Retrieve unknown adaption type '" << adaption_type << "'!");
//      return false;
//    }
//  }
//  else
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/adaption_type' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_frame", mo_adapt_params.input_ref_frame))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_frame' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_dep_parent", mo_adapt_params.input_ref_dep_parent))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_dep_parent' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_dep_child", mo_adapt_params.input_ref_dep_child))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_dep_child' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_correction/roll", mo_adapt_params.input_ref_correction.roll))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_correction/roll' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_correction/pitch", mo_adapt_params.input_ref_correction.pitch))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_correction/pitch' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_ref_correction/yaw", mo_adapt_params.input_ref_correction.yaw))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_ref_correction/yaw' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/target_ref_frame", mo_adapt_params.target_ref_frame))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/target_ref_frame' from parameter server!");
//    return false;
//  }
  XmlRpc::XmlRpcValue endpts_list;
//  if(!nh_private.getParam("motion_adaption/input_endpts", endpts_list))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_endpts' from parameter server!");
//    return false;
//  }
//  else
//  {
//    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
//    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
//    {
//      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeString);
//      mo_adapt_params.input_endpts.push_back(std::string(endpts_list[endpt]));
//    }
//  }
//  if(!nh_private.getParam("motion_adaption/target_endpts", endpts_list))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/target_endpts' from parameter server!");
//    return false;
//  }
//  else
//  {
//    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
//    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
//    {
//      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeString);
//      mo_adapt_params.target_endpts.push_back(std::string(endpts_list[endpt]));
//    }
//  }
//  if(!nh_private.getParam("motion_adaption/goal_endpts", endpts_list))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/goal_endpts' from parameter server!");
//    return false;
//  }
//  else
//  {
//    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
//    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
//    {
//      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeString);
//      mo_adapt_params.goal_endpts.push_back(std::string(endpts_list[endpt]));
//    }
//  }
//  if(!nh_private.getParam("motion_adaption/input_correction/roll", mo_adapt_params.input_correction.roll))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_correction/roll' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_correction/pitch", mo_adapt_params.input_correction.pitch))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_correction/pitch' from parameter server!");
//    return false;
//  }
//  if(!nh_private.getParam("motion_adaption/input_correction/yaw", mo_adapt_params.input_correction.yaw))
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/input_correction/yaw' from parameter server!");
//    return false;
//  }
//  mo_ret_params.motion_adaption_parameters.push_back(mo_adapt_params);

//
//  /*
//   * tree kinematics configuration
//   */
  tree_kinematics::KinematicsParameters kinematics_params;
  if(!nh_private.getParam("kinematics/robot_model_description_name", kinematics_params.robot_model_description_name))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/robot_model_description_name' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/endpt_names", endpts_list))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/endpt_names' from parameter server!");
    return false;
  }
  else
  {
    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
    {
      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeString);
      kinematics_params.endpt_names.push_back(std::string(endpts_list[endpt]));
    }
  }
  if(!nh_private.getParam("kinematics/epsilon", kinematics_params.epsilon))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/epsilon' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/max_iterations", kinematics_params.max_iterations))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/max_iterations' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/lambda", kinematics_params.lambda))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/lambda' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/ik_call_frequency", kinematics_params.ik_call_frequency))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/ik_call_frequency' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/q_dot_max_factor", kinematics_params.q_dot_max_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/q_dot_max_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/q_dot_min_factor", kinematics_params.q_dot_min_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/q_dot_min_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/x_dot_trans_max_factor", kinematics_params.x_dot_trans_max_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/x_dot_trans_max_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/x_dot_trans_min_factor", kinematics_params.x_dot_trans_min_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/x_dot_trans_min_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/x_dot_rot_max_factor", kinematics_params.x_dot_rot_max_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/x_dot_rot_max_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/x_dot_rot_min_factor", kinematics_params.x_dot_rot_min_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/x_dot_rot_min_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/low_pass_factor", kinematics_params.low_pass_factor))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/low_pass_factor' from parameter server!");
    return false;
  }
  if(!nh_private.getParam("kinematics/task_space_weights", endpts_list))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/task_space_weights' from parameter server!");
    return false;
  }
  else
  {
    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
    {
      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::vector<double> weights;
      for (int weight = 0; weight < endpts_list[endpt].size(); ++weight)
      {
        ROS_ASSERT(endpts_list[endpt][weight].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        weights.push_back(double(endpts_list[endpt][weight]));
      }
      kinematics_params.task_space_weights.push_back(weights);
    }
  }
  if(!nh_private.getParam("kinematics/joint_space_weights", endpts_list))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/joint_space_weights' from parameter server!");
    return false;
  }
  else
  {
    ROS_ASSERT(endpts_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int endpt = 0; endpt < endpts_list.size(); ++endpt)
    {
      ROS_ASSERT(endpts_list[endpt].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      kinematics_params.joint_space_weights.push_back(double(endpts_list[endpt]));
    }
  }
//  mo_ret_params.kinematics_parameters = kinematics_params;
  mo_ret_conf = MoRetConfPtr(new MoRetConf(mo_ret_general_params, mo_adapt_params, kinematics_params));
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh, nh_private("~");
  ROS_INFO_STREAM("Initialising controller. [" << node_name << "]");
//  MoRetParams mo_ret_params;
  MoRetConfPtr mo_ret_conf;
  MoRetPtr mo_ret;
  if (createMotionRetargetingConfiguration(nh_private, mo_ret_conf))
  {
    ROS_INFO_STREAM("Motion retargeting configuration created. [" << node_name << "]");
    try
    {
      mo_ret = MoRetPtr(new MoRet(*mo_ret_conf, nh));
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

//  if (createMotionRetargetingFromParameters(nh_private, mo_ret_params))
//  {
//    ROS_INFO_STREAM("Motion retargeting parameters retrieved. [" << node_name << "]");
//    try
//    {
//      mo_ret_conf = MoRetConfPtr(new MoRetConf(mo_ret_params));
//      ROS_INFO_STREAM("Motion retargeting configuration created. [" << node_name << "]");
//      mo_ret = MoRetPtr(new MoRet(*mo_ret_conf, nh));
//      ROS_INFO_STREAM("Motion retargeting ready to rock! [" << node_name << "]");
//    }
//    catch (std::exception& e)
//    {
//      ROS_ERROR_STREAM("Caught an exception, while trying to initialise motion retargeting: "
//                       << e.what());
//      ROS_ERROR_STREAM("Aborting.");
//      return -1;
//    }
//  }
//  else
//  {
//    ROS_ERROR_STREAM("Couldn't retrieve all retargeting parameters! Aborting. [" << node_name << "]");
//    return -1;
//  }
  trajectory_msgs::JointTrajectoryPoint output_joint_states;
  ros::Rate loop_rate(mo_ret_conf->getGeneralParameters().retargeting_freq);
  while (nh.ok())
  {
    ros::spinOnce();
    try
    {
      // Get rid of exceptions here - apparently its computational expensive.
      ROS_INFO_STREAM_THROTTLE(1.0, "Retargeting ... [" << node_name << "]");
      mo_ret->retarget();
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Caught this exception, while trying to do retargeting: " << e.what());
      return -1;
    }
    loop_rate.sleep();
  }
  ROS_INFO_STREAM("Finished motion retargeting. Exiting.");
  return 0;
}
