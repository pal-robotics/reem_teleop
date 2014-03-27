/**
 * License: BSD
 *      https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <string>
#include <exception>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <motion_retargeting/motion_recorder/rosbagger.h>
#include <motion_retargeting/output_handler/follow_joint_trajectory_action_output_handler.h>
#include <motion_retargeting_msgs/Motion.h>
#include <motion_retargeting_msgs/MotionList.h>
#include <motion_retargeting_msgs/MotionPlayback.h>
#include <motion_retargeting_msgs/PlaybackStatus.h>
#include <motion_retargeting_msgs/PlaybackStatusCodes.h>
#include <motion_retargeting_msgs/utils.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>

namespace bf = boost::filesystem;

namespace Statuses
{
  enum Status
  {
    IDLING = 0,
    PLAYBACK = 1,
  };
}
int status = Statuses::IDLING;
bool new_motion_playback = false;
bool stop_playback = false;
std::string new_motion_name("");
std::map<std::string, std::string> motion_list;


void motionPlaybackCB(const motion_retargeting_msgs::MotionPlayback::ConstPtr& msg)
{
  if (msg->start_playback)
  {
    if (motion_list.find(msg->motion_name) == motion_list.end())
    {
      ROS_WARN_STREAM("Motion Player: Requested motion '" + msg->motion_name + "' unknown.");
      return;
    }
    else
    {
      new_motion_playback = true;
      new_motion_name = msg->motion_name;
    }

    if (status == Statuses::PLAYBACK)
    {
      ROS_INFO_STREAM("Motion Player: Request for playing back motion '" + new_motion_name + "' recevied,"
                      << " but currently playing back a motion."
                      << " Will stop current motion playback and re-start with new motion.");
    }
    else
    {
      ROS_INFO_STREAM("Motion Player: Request for playing back motion '" + new_motion_name + "'.");
    }
  }
  else
  {
    if (status == Statuses::PLAYBACK)
    {
      stop_playback = true;
      ROS_WARN_STREAM("Motion Player: Request for stopping motion playback received. ");
    }
  }
  return;
}


int main (int argc, char** argv)
{

  ros::init(argc, argv, "motion_player");
  ros::NodeHandle nh, nh_private("~");
  std::string node_name = ros::this_node::getName();
  ros::Subscriber sub_motion_playback = nh_private.subscribe("motion_playback", 1, &motionPlaybackCB);
  ros::Publisher pub_motion_playback = nh_private.advertise<motion_retargeting_msgs::MotionList>("motion_list", 1,
                                                                                                 true);
  ros::Publisher pub_playback_status = nh_private.advertise<motion_retargeting_msgs::PlaybackStatus>("playback_status",
                                                                                                     1, true);
  motion_retargeting_msgs::PlaybackStatus playback_status_msg;

   // get parameters
  std::string action_server_name;
  if(!nh_private.getParam("action_server_name", action_server_name))
  {
    ROS_ERROR_STREAM("Couldn't get parameter 'action_server_name'. Aborting. [" << node_name << "]");
    return -1;
  }

  motion_retargeting::FollowJointTrajectoryActionHandler output_handler(action_server_name);
  if(output_handler.init())
  {
    ROS_INFO_STREAM("Output handler initialised. [" << node_name << "]");
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise output handler. Aborting. [" << node_name << "]");
    return -2;
  }

  std::string motion_package;
  if(!nh_private.getParam("motion_package", motion_package))
  {
    ROS_ERROR_STREAM("Couldn't get parameter 'motion_package'. Aborting. [" << node_name << "]");
    return -3;
  }

  // crawl motion package for motions
  motion_retargeting_msgs::MotionList motion_list_msg;
  std::string motion_package_path = ros::package::getPath(motion_package);
  if (motion_package_path == "")
  {
    ROS_ERROR_STREAM("Couldn't find package '" +  motion_package + "'. Aborting. [" << node_name << "]");
    return -4;
  }
  else
  {
    ROS_INFO_STREAM("Crawling package '" +  motion_package + "' for motions. [" << node_name << "]");
    bf::path current_dir(motion_package_path);

    for (bf::recursive_directory_iterator iter(current_dir), end; iter != end; ++iter)
    {
      if (iter->path().extension() == ".bag")
      {
        std::string name_plus_ext = iter->path().string().substr(iter->path().string().find_last_of("/") + 1);
        std::string name = name_plus_ext.substr(0, name_plus_ext.find_last_of("."));
        motion_retargeting_msgs::Motion motion_msg;
        motion_msg.motion_name = name;
        motion_list_msg.motions.push_back(motion_msg);
        motion_list.insert(std::pair<std::string, std::string>(name, iter->path().string()));
      }
    }
    ROS_INFO_STREAM("Added " << motion_list_msg.motions.size() << " motion(s) to the list. [" << node_name << "]");
  }
  pub_motion_playback.publish(motion_list_msg);

  motion_retargeting::Rosbagger rosbagger;
  ros::Duration time_step;
  double avg_freq = 0.0;
  unsigned int pub_count = 0;
  ros::Time last_stamp(0.0);
  sensor_msgs::JointState joint_states;
  bool status_message_published = false;

  while (ros::ok())
  {
    switch (status)
    {
      case Statuses::IDLING:
        if (!status_message_published)
        {
          // publish status message
          playback_status_msg.status_code.value = motion_retargeting_msgs::PlaybackStatusCodes::IDLING;
          playback_status_msg.status_message =
              motion_retargeting_msgs::errorCodeToString(playback_status_msg.status_code);
          pub_playback_status.publish(playback_status_msg);
          status_message_published = true;
        }
        if (new_motion_playback)
        {
          new_motion_playback = false;
          if (rosbagger.preparePlayback(motion_list.find(new_motion_name)->second))
          {
            ROS_INFO_STREAM("Ready to play back new motion '" + new_motion_name + "'. [" << node_name << "]");
            status = Statuses::PLAYBACK;
            status_message_published = false;
          }
          else
          {
            ROS_ERROR_STREAM("Error occurred while preparing for motion playback. [" << node_name << "]");
            // publish status message
            playback_status_msg.status_code.value = motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK_ERROR;
            pub_playback_status.publish(playback_status_msg);
          }
        }
        break;
      case Statuses::PLAYBACK:
        if (!status_message_published)
        {
          // publish status message
          playback_status_msg.status_code.value = motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK;
          playback_status_msg.status_message =
              motion_retargeting_msgs::errorCodeToString(playback_status_msg.status_code);
          pub_playback_status.publish(playback_status_msg);
          ROS_INFO_STREAM("Playing back motion. [" << node_name << "]");
          status_message_published = true;
        }
        if(rosbagger.readMotion(joint_states))
        {
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
          ROS_DEBUG_STREAM_THROTTLE(1.0, "Current/Average publishing frequency: " << (1/time_step.toSec()) << " / "
                                   << avg_freq << " [" << node_name << "]");
        }
        else
        {
          ROS_INFO_STREAM("All messages of the rosbag have been published. [" << node_name << "]");
          stop_playback = true;
        }

        if (stop_playback)
        {
          rosbagger.stopPlayback();
          ROS_INFO_STREAM("Playback stopped. [" << node_name << "]");
          avg_freq = 0.0;
          pub_count = 0;
          last_stamp = ros::Time(0.0);
          stop_playback = false;
          // publish status message
          playback_status_msg.status_code.value = motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK_FINISHED;
          playback_status_msg.status_message =
              motion_retargeting_msgs::errorCodeToString(playback_status_msg.status_code);
          pub_playback_status.publish(playback_status_msg);
          status = Statuses::IDLING;
          status_message_published = false;
        }
        break;
    }
    ros::spinOnce();
  }
  return 0;
}
