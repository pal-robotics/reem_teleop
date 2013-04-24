/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/rosbagger.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 24, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ROSBAGGER_H_
#define ROSBAGGER_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <iostream>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "motion_recorder.h"


namespace motion_retargeting
{

/**
 * A motion recorder, which uses rosbag to stores incoming joint states in a bag file
 */
class Rosbagger : public MotionRecorder
{
public:
  Rosbagger(const rosbag::BagMode& bagmode,
             const std::string& filename = std::string("")) :
             MotionRecorder(),
             bagmode_(bagmode),
             bag_name_(filename)
  {
    recorded_motion_topic_name_ = "recorded_motion";
    if (bagmode_ == rosbag::bagmode::Write)
    {
      std::stringstream ss;
      ss << to_iso_string(ros::Time::now().toBoost()) << "_motion_retargeting_record.bag";
      ss >> bag_name_;
      try
      {
        bag_.open(bag_name_, rosbag::bagmode::Write);
        ROS_INFO_STREAM("Rosbagger: Ready to write to rosbag '" << bag_name_ << "'.");
      }
      catch (rosbag::BagException& e)
      {
        ROS_ERROR_STREAM("Rosbagger: Exception thrown while creating rosbag '" << bag_name_ << "'!");
        throw std::invalid_argument(bag_name_);
      }
    }
    else if (bagmode_ == rosbag::bagmode::Read)
    {
      if (bag_name_ != std::string(""))
      {
        try
        {
          bag_.open(bag_name_, rosbag::bagmode::Read);
        }
        catch (rosbag::BagException& e)
        {
          ROS_ERROR_STREAM("Rosbagger: Exception thrown while opening rosbag '" << bag_name_ << "'!");
          throw std::invalid_argument(bag_name_);
          return;
        }
        view_ptr_ = boost::shared_ptr<rosbag::View>(new rosbag::View(bag_,
                                                                     rosbag::TopicQuery(recorded_motion_topic_name_)));
        view_it_ = view_ptr_->begin();
        ROS_INFO_STREAM("Rosbagger: Ready to read from rosbag '" << bag_name_ << "'.");
      }
      else
      {
        ROS_ERROR_STREAM("Rosbagger: Initialsing in read mode, but no filename was provided!");
        throw std::invalid_argument(bag_name_);
      }
    }
    else if (bagmode_ == rosbag::bagmode::Append)
    {
      std::stringstream ss;
      ss << bagmode;
      std::string invalid_arg;
      ss >> invalid_arg;
      ROS_ERROR_STREAM("Rosbagger: Appending is not supported!");
      throw std::invalid_argument(invalid_arg);
    }
    else
    {
      std::stringstream ss;
      ss << bagmode;
      std::string invalid_arg;
      ss >> invalid_arg;
      ROS_ERROR_STREAM("Rosbagger: Unknown bagmode (" << invalid_arg << ") received.");
      throw std::invalid_argument(invalid_arg);
    }
  };

  ~Rosbagger()
  {
    bag_.close();
    std::cout << "Rosbagger: Rosbag '" << bag_name_ << "' closed." << std::endl;
  };

  virtual bool storeMotion(const sensor_msgs::JointState& joint_states)
  {
    if(bagmode_ == rosbag::bagmode::Write)
    {
      try
      {
        bag_.write(recorded_motion_topic_name_, ros::Time::now(), joint_states);
        return true;
      }
      catch (rosbag::BagIOException& e)
      {
        ROS_WARN_STREAM("Rosbagger: An error occured when trying to write to rosbag.");
        ROS_DEBUG_STREAM(e.what());
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't write to rosbag, since rosbagger is in read mode!");
      return false;
    }
  };

  virtual bool readMotion(sensor_msgs::JointState& joint_states)
  {
    if(bagmode_ == rosbag::bagmode::Read)
    {
      if (view_it_ != view_ptr_->end())
      {
        sensor_msgs::JointState::ConstPtr js_ptr = view_it_->instantiate<sensor_msgs::JointState>();
        if (js_ptr != NULL)
        {
          joint_states = *js_ptr;
          ROS_DEBUG_STREAM("Rosbagger: Retrieved this joint state message from the rosbag: "
                           << joint_states);
          view_it_++;
          return true;
        }
        else
        {
          ROS_WARN_STREAM("Rosbagger: Couldn't retrieve joint state message from rosbag!");
          return false;
        }
      }
      ROS_WARN_STREAM("Rosbagger: We reached the end of the rosbag.");
      return false;
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't read from rosbag, since rosbagger is in write mode!");
      return false;
    }
  };

private:
  /**
   * The rosbag to write data to/read data from
   */
  rosbag::Bag bag_;
  /**
   * Pointer to a rosbag view for reading from the rosbag
   */
  boost::shared_ptr<rosbag::View> view_ptr_;
  /**
   * Rosbag view iterator for retrieving messages from the rosbag
   */
  rosbag::View::iterator view_it_;
  /**
   * Mode of the rosbagger - either read or write
   */
  rosbag::BagMode bagmode_;
  /**
   * Filename of the rosbag
   */
  std::string bag_name_;
  /**
   * Name of the topic the recorded motion is/will be stored at
   */
  std::string recorded_motion_topic_name_;
};

typedef boost::shared_ptr<Rosbagger> RosbaggerPtr;

} // namespace motion_retargeting

#endif /* ROSBAGGER_H_ */
