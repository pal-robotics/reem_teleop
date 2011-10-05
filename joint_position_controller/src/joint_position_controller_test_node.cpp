/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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

 
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_position_controller_test_node");
  ros::NodeHandle nh;
  
  // publisher for joint states commands
  ros::Publisher pub_joint_states_cmd = nh.advertise<sensor_msgs::JointState>("/joint_states_cmd", 1);
  sensor_msgs::JointState joint_states_cmd;

  unsigned int switcher = 0;
  ros::Rate loop_rate(0.5);
  
  while(nh.ok()) 
  {
    joint_states_cmd.position.clear();

    switch (switcher)
    {
      case 0:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.0);
        }
        switcher = 1;
        break;
      
      case 1:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.2);
        }
        switcher = 2;
        break;
        
      case 2:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.5);
        }
        switcher = 3;
        break;
        
       case 3:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.2);
        }
        switcher = 4;
        break;
        
      case 4:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.1);
        }
        switcher = 0;
        break;
        
      default:
        for(unsigned int i = 0; i < 24; ++i)
        {
          joint_states_cmd.position.push_back(0.0);
        }
        switcher = 0;
    }
    joint_states_cmd.header.stamp = ros::Time::now();
    pub_joint_states_cmd.publish(joint_states_cmd);
    
    loop_rate.sleep();
  }
  return 0;
}
