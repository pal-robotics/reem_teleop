/**
 * @file /motion_retargeting_controller/src/robosem_motion_playback.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 24, 2013
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <exception>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>
#include "motion_retargeting_controller/rosbagger.h"
#include "motion_retargeting_controller/follow_joint_trajectory_action_output_handler.h"

namespace po = boost::program_options;

int main (int argc, char** argv)
{
  std::string filename("");
  std::string output_topic("");
  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("filename,f", po::value<std::string>(), "filename of the stored motion (rosbag)")
        ("output_topic,o", po::value<std::string>(), "name of the action server motion commands will be sent to")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }
    if (vm.count("filename"))
    {
      filename = vm["filename"].as<std::string>();
      std::cout << "Will read data from " << filename << "." << std::endl;
    }
    else
    {
      std::cout << "No filename provided. Aborting." << std::endl;
      return -1;
    }
    if (vm.count("output_topic"))
    {
      output_topic = vm["output_topic"].as<std::string>();
      std::cout << "Will send motion commands to action server '" << filename << "'." << std::endl;
    }
  }
  catch(std::exception& e)
  {
      std::cerr << "An error occurred will parsing the program's arguments: " << e.what() << std::endl;
      return -2;
  }
  catch(...)
 {
      std::cerr << "An unknown exception was thrown will parsing the program's arguments!" << std::endl;
      return -3;
  }

  std::string node_name("robosem_motion_playback");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::Duration time_step;
  double avg_freq = 0.0;
  unsigned int pub_count = 0;
  ros::Time last_stamp(0.0);
  sensor_msgs::JointState joint_states;
  motion_retargeting::RosbaggerPtr rosbagger;
  try
  {
    rosbagger = motion_retargeting::RosbaggerPtr(new motion_retargeting::Rosbagger(rosbag::bagmode::Read, filename));
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Robosem Motion Playback: An exception was thrown when trying to initialise the rosbagger. ["
                     << node_name << "]");
    ROS_DEBUG_STREAM(e.what());
    return -4;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("An unknown exception was thrown when trying to initialise the rosbagger! [" << node_name << "]");
    return -5;
  }
  motion_retargeting::FollowJointTrajectoryActionHandler output_handler(output_topic);

  while (ros::ok())
  {
    if(!rosbagger->readMotion(joint_states))
    {
      ROS_INFO_STREAM("All messages of the rosbag have been published. [" << node_name << "]");
      return 0;
    }
    if (last_stamp > ros::Time(0.0))
    {
      time_step = ros::Duration(joint_states.header.stamp - last_stamp);
      avg_freq = ((avg_freq * (pub_count - 1)) + time_step.toSec()) / pub_count;
    }
    else
    {
      time_step = ros::Duration(0.0);
    }
    last_stamp = joint_states.header.stamp;
    pub_count++;
    time_step.sleep();
    output_handler.setOutput(joint_states);
    ROS_INFO_STREAM_THROTTLE(1.0, "Current/Average publishing frequency: " << (1/time_step.toSec()) << " / " << avg_freq << " [" << node_name << "]");
    ros::spinOnce();
  }
}
