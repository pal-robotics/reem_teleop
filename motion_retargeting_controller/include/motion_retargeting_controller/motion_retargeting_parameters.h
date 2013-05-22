/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/motion_retargeting_parameters.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date May 16, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_RETARGETING_PARAMETERS_H_
#define MOTION_RETARGETING_PARAMETERS_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <motion_adaption/types/adaption_parameters.h>
#include <tree_kinematics/kinematics_parameters.h>


/**
 * Get the parameters from the parameter server and set up the motion retargeting configuration
 */
bool getMotionRetargetingParameters(const ros::NodeHandle&  nh_private,
                                        double& retargeting_frequency,
                                        std::vector<motion_adaption::AdaptionParameters*>& mo_adapt_params,
                                        tree_kinematics::KinematicsParameters& kinematics_params)
{
  /*
   * General parameters
   */
  if(!nh_private.getParam("retargeting_frequency", retargeting_frequency))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'retargeting_frequency' from parameter server!");
    return false;
  }
  /*
   * motion adaption configuration
   */
  // First, check how many adaption shall be configured
  int nr_of_adaptions = 0;
  if(!nh_private.getParam("motion_adaption/nr_of_adaptions", nr_of_adaptions))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'motion_adaption/nr_of_adaptions' from parameter server!");
    return false;
  }
  // Then retrieve the common and the adaption-specific parameters for each adaption
  for (unsigned int adaption = 1; adaption <= nr_of_adaptions; ++adaption)
  {
    // stringstream magic to ease parameter retrieval
    std::stringstream ss;
    std::string parameter_prep, parameter_name;
    ss << "motion_adaption/adaption_" << adaption  << "/";
    ss >> parameter_prep;

    // Retrieve common parameters
    motion_adaption::AdaptionParameters mo_adapt_common_params;
    ss.clear();
    ss << parameter_prep << "adaption_name";
    ss >> parameter_name;
    if (!nh_private.getParam(parameter_name, mo_adapt_common_params.adaption_name))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "adaption_type";
    ss >> parameter_name;
    std::string adaption_type;
    if (nh_private.getParam(parameter_name, adaption_type))
    {
      if(adaption_type == "trans_rot_adaption")
      {
        mo_adapt_common_params.adaption_type = motion_adaption::AdaptionParameters::TransRotAdaption;
      }
      else if (adaption_type == "hands_adaption")
      {
        mo_adapt_common_params.adaption_type = motion_adaption::AdaptionParameters::HandsAdaption;
      }
      else
      {
        ROS_ERROR_STREAM("Unknown adaption type '" << adaption_type << "' received!");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "input_ref_name";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.input_ref_name))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "input_pos_ref_name";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.input_pos_ref_name))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "input_ref_orient_adjust/roll";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.input_ref_orient_adjust.roll))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep <<"input_ref_orient_adjust/pitch";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.input_ref_orient_adjust.pitch))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "input_ref_orient_adjust/yaw";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.input_ref_orient_adjust.yaw))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    ss.clear();
    ss << parameter_prep << "target_ref_name";
    ss >> parameter_name;
    if(!nh_private.getParam(parameter_name, mo_adapt_common_params.target_ref_name))
    {
      ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
      return false;
    }
    // Retrieve adaption-specific parameters
    if (adaption_type == "trans_rot_adaption")
    {
      motion_adaption::TransRotAdaptionParameters* trans_rot_adapt_params(
          new motion_adaption::TransRotAdaptionParameters(mo_adapt_common_params));
      ss.clear();
      ss << parameter_prep << "input_endpt_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->input_endpt_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_endpt_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->target_endpt_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_endpt_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->goal_endpt_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_orient_adjust/roll";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->goal_orient_adjust.roll))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep <<"goal_orient_adjust/pitch";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->goal_orient_adjust.pitch))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_orient_adjust/yaw";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, trans_rot_adapt_params->goal_orient_adjust.yaw))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      mo_adapt_params.push_back(trans_rot_adapt_params);
    }
    else if (adaption_type == "hands_adaption")
    {
      motion_adaption::HandsAdaptionParameters* hands_adapt_params(
                new motion_adaption::HandsAdaptionParameters(mo_adapt_common_params));
      ss.clear();
      ss << parameter_prep << "input_torso_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_torso_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_r_shoulder_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_r_shoulder_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_r_elbow_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_r_elbow_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_r_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_r_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_l_shoulder_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_l_shoulder_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_l_elbow_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_l_elbow_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "input_l_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->input_l_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_torso_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_torso_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_r_shoulder_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_r_shoulder_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_r_elbow_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_r_elbow_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_r_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_r_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_l_shoulder_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_l_shoulder_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_l_elbow_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_l_elbow_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "target_l_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->target_l_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_r_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_r_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_r_hand_orient_adjust/roll";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_r_hand_orient_adjust.roll))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep <<"goal_r_hand_orient_adjust/pitch";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_r_hand_orient_adjust.pitch))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_r_hand_orient_adjust/yaw";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_r_hand_orient_adjust.yaw))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_l_hand_name";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_l_hand_name))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_l_hand_orient_adjust/roll";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_l_hand_orient_adjust.roll))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep <<"goal_l_hand_orient_adjust/pitch";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_l_hand_orient_adjust.pitch))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      ss.clear();
      ss << parameter_prep << "goal_l_hand_orient_adjust/yaw";
      ss >> parameter_name;
      if(!nh_private.getParam(parameter_name, hands_adapt_params->goal_l_hand_orient_adjust.yaw))
      {
        ROS_ERROR_STREAM("Couldn't retrieve parameter " << parameter_name << " from parameter server!");
        return false;
      }
      mo_adapt_params.push_back(hands_adapt_params);
    }
    else
    {
      ROS_ERROR_STREAM("Retrieve unknown adaption type '" << adaption_type << "'!");
      return false;
    }
  }
  /*
   * tree kinematics configuration
   */
//  tree_kinematics::KinematicsParameters kinematics_params;
  if(!nh_private.getParam("kinematics/robot_model_description_name", kinematics_params.robot_model_description_name))
  {
    ROS_ERROR_STREAM("Couldn't retrieve parameter 'kinematics/robot_model_description_name' from parameter server!");
    return false;
  }
  XmlRpc::XmlRpcValue endpts_list;
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
  return true;
}

#endif /* MOTION_RETARGETING_PARAMETERS_H_ */
