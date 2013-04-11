/*
 * Software License Agreement (GNU Lesser General Public License)
 *
 * Copyright (c) 2011, PAL Robotics, S.L.
 * All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
/**
 * \author Marcus Liebhardt
 * \copyright LPGL
 */


#include "tree_kinematics/tree_kinematics.h"


/**
 * \brief Initialising the tree_kinematics class
 *
 * This node initialises the tree_kinematics class in order to offer the inverse and forward kinematics services.
 *
 * @return -1 if tree_kinematics class could not be initialised and 0 if program was stopped manually.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tree_kinematics_node");
  ros::NodeHandle nh;
  tree_kinematics::TreeKinematicsPtr tree_kinematics;
  tree_kinematics::KinematicsParameters parameters;
  ros::ServiceServer fk_service, ik_service;
  try
  {
    tree_kinematics = tree_kinematics::TreeKinematicsPtr(new tree_kinematics::TreeKinematics(parameters, nh));
  }
  catch (...)
  {
    ROS_FATAL("Could not initialise tree kinematics! Aborting ...");
    return -1;
  }

  fk_service = nh.advertiseService("get_position_fk",
                                   &tree_kinematics::TreeKinematics::getPositionFk,
                                   tree_kinematics);
  ik_service = nh.advertiseService("get_position_ik",
                                   &tree_kinematics::TreeKinematics::getPositionIk,
                                   tree_kinematics);
  ROS_INFO_STREAM("Tree kinematics services initialised.");

  ros::spin();

  return 0;
}
