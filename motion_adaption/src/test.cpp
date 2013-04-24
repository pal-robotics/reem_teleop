/**
 * @file /motion_adaption/src/test.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 20, 2013
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "motion_adaption/motion_adaption.h"

using namespace motion_adaption;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_motion_adaption");
  std::vector<AdaptionParameters*> mo_adapt_params;
  TransRotAdaptionParameters* trans_rot_adapt_params(new TransRotAdaptionParameters);
  trans_rot_adapt_params->adaption_name = "test_trans_rot_adaption";
  trans_rot_adapt_params->adaption_type = AdaptionParameters::TransRotAdaption;
  trans_rot_adapt_params->input_endpt_name = "test_input_endpt_name";
  mo_adapt_params.push_back(trans_rot_adapt_params);
  HandsAdaptionParameters* hands_adapt_params(new HandsAdaptionParameters);
  hands_adapt_params->adaption_name = "test_hands_adaption";
  hands_adapt_params->adaption_type = AdaptionParameters::HandsAdaption;
  hands_adapt_params->input_l_elbow_name = "test_input_l_elbow_name";
  mo_adapt_params.push_back(hands_adapt_params);
//  MotionAdaption motion_adaption(mo_adapt_params);
  MotionAdaptionPtr motion_adaption = MotionAdaptionPtr(new MotionAdaption(mo_adapt_params));

  std::vector<geometry_msgs::PoseStamped> adapted_poses_output;
//  motion_adaption.adapt(adapted_poses_output);
  motion_adaption->adapt(adapted_poses_output);

  delete trans_rot_adapt_params;
  return 0;
}
