/*
 * Filename: motion_adaption_node.cpp
 * Author: Marcus Liebhardt
 * Date: XX.04.2011
 * License: TODO
 */

//#include <cmath>

#include <ros/ros.h>
//#include <std_msgs/Float64MultiArray.h>


#include "motion_adaption/motion_adaption.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_adaption_node");
  
  ros::NodeHandle nh;
  ros::Rate loop_rate(30.0);
  unsigned int loop_count = 1;
  double cycle_time_median = 0.0;
  
  MotionAdaption motion_adaption;

  while (nh.ok())
  {
    motion_adaption.adapt();

    loop_rate.sleep();
    
    cycle_time_median = ((cycle_time_median * (loop_count - 1)) + loop_rate.cycleTime().toSec()) / loop_count;

    ROS_INFO_THROTTLE(1.0, "motion_adaption: cycle time %f and median cycle time %f", loop_rate.cycleTime().toSec(),
    cycle_time_median);

    loop_count++;
  }

  ROS_INFO("exit");
  return 0;
}

