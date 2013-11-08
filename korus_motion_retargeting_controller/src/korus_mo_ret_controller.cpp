/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/korus_motion_retargeting_controller/LICENSE
 **/

#include <string>
#include <exception>
#include <ros/ros.h>
#include <motion_adaption/motion_adaption.h>
#include <motion_adaption/types/adaption_type.h>
#include <motion_retargeting/motion_retargeting.h>
#include <motion_retargeting/motion_retargeting_parameters.h>
#include <motion_retargeting/motion_recorder/rosbagger.h>
#include <motion_retargeting/output_handler/follow_joint_trajectory_action_output_handler.h>
#include <tree_kinematics/tree_kinematics.h>

// Globals
std::string node_name = "korus_mo_ret_controller";

typedef motion_retargeting::MotionRetargeting MoRet;
typedef motion_retargeting::MotionRetargetingPtr MoRetPtr;


int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh, nh_private("~");
  ROS_INFO_STREAM("Initialising controller. [" << node_name << "]");

  motion_adaption::MotionAdaptionPtr motion_adaption;
  tree_kinematics::TreeKinematicsPtr tree_kinematics;
  motion_retargeting::MotionRecorderPtr motion_recorder;
  motion_retargeting::OutputHandlerPtr output_handler;
  motion_retargeting::MotionRetargetingPtr motion_retargeting;

  double retargeting_frequency = 0.0;
  std::vector<motion_adaption::AdaptionParameters*> motion_adaption_params;
  tree_kinematics::KinematicsParameters kinematics_params;

  if (!getMotionRetargetingParameters(nh_private,
                                      retargeting_frequency,
                                      motion_adaption_params,
                                      kinematics_params))
  {
    ROS_ERROR_STREAM("Failed to create the motion retargeting configuration. Aborting.");
    return -1;
  }
  ROS_INFO_STREAM("Motion retargeting configuration created. [" << node_name << "]");
  try
  {
    // Initialise motion adaption
    motion_adaption = motion_adaption::MotionAdaptionPtr(
                      new motion_adaption::MotionAdaption(motion_adaption_params));
    // Initialise tree kinematics
    tree_kinematics = tree_kinematics::TreeKinematicsPtr(
                      new tree_kinematics::TreeKinematics(kinematics_params, nh));
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
  return 0;
}
