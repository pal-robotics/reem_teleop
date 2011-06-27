/*
 * Filename: tree_kinematics_node.cpp
 * Author: Marcus Liebhardt
 * Created: 18.03.2011
 * License:
 * Description:
 */

#include "tree_kinematics/tree_kinematics.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree_kinematics_node");
  ros::NodeHandle nh;
  tree_kinematics::TreeKinematics tree_kinematics;
  
  if(!tree_kinematics.init())
  {
    ROS_ERROR("Could not initialize tree kinematics node!");
    return -1;
  }
  else
  {
    ROS_INFO("Tree kinematics node initialized.");
    ros::spin();
  }
  return 0;
}

