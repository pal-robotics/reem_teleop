/*
 * Filename: tree_kinematics_node.cpp
 * Author: Marcus Liebhardt
 * Created: 18.03.2011
 * Description:
 */

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

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

