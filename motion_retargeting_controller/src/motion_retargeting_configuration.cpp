/**
 * @file /motion_retargeting_controller/src/motion_retargeting_configuration.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 9, 2013
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "motion_retargeting_controller/motion_retargeting_configuration.h"

namespace motion_retargeting
{

MotionRetargetingConfiguration::MotionRetargetingConfiguration(
    MotionRetargetingParameters motion_retargeting_parameters)
{
  parameters_ = motion_retargeting_parameters;
//  general_config_param_.retargeting_freq = retargeting_freq;
//  general_config_param_.wait_for_tf = wait_for_tf;
//  general_config_param_.epsilon = epsilon;
//  general_config_param_.max_iterations = max_iterations;
//  general_config_param_.q_dot_min_factor = q_dot_min_factor;
//  general_config_param_.q_dot_max_factor = q_dot_max_factor;
//  general_config_param_.low_pass_factor = low_pass_factor;
//  general_config_param_.x_dot_trans_min = x_dot_trans_min;
//  general_config_param_.x_dot_trans_max = x_dot_rot_min;
//  general_config_param_.x_dot_rot_min = x_dot_rot_min;
//  general_config_param_.x_dot_rot_max = x_dot_rot_max;
//  general_config_param_.lambda = lamda;
//  general_config_param_.robot_model_name = robot_model_name;

//  unsigned int nr_of_retargetings = retargetings.size();
//  for (unsigned int retargeting; retargeting < nr_of_retargetings; ++ retargeting)
//  {
//    retargetings_.push_back(retargetings[retargeting]);
//  }
}

MotionRetargetingConfiguration::~MotionRetargetingConfiguration(){};

} // namespace motion_retargeting
