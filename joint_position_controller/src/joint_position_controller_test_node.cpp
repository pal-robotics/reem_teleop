/*
 * Filename: joint_position_controller_test_node.cpp
 * Author: Marcus Liebhardt
 * Created: 22.03.2011
 * License:
 */
 
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
