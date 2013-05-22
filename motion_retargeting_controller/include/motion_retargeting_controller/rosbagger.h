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
  Rosbagger() : MotionRecorder(),
                 recording_(false),
                 playback_(false)
  {
    recorded_motion_topic_name_ = "recorded_motion";
  };

  ~Rosbagger()
  {
    // Just to make sure the bag is closed
    bag_.close();
  };

  virtual bool prepareRecording()
  {
    if (!playback_)
    {
      std::stringstream ss;
      ss << to_iso_string(ros::Time::now().toBoost()) << "_motion_retargeting_record.bag";
      ss >> bag_name_;
      try
      {
        bag_.open(bag_name_, rosbag::bagmode::Write);
        ROS_INFO_STREAM("Rosbagger: Ready to write to rosbag '" << bag_name_ << "'.");
        recording_ = true;
      }
      catch (rosbag::BagException& e)
      {
        ROS_ERROR_STREAM("Rosbagger: Exception thrown while creating rosbag '" << bag_name_ << "'!");
        throw std::invalid_argument(bag_name_);
      }
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't prepare recording, since rosbagger is in playback mode.");
      return false;
    }
  };

  virtual bool stopRecording()
  {
    if (recording_)
    {
      bag_.close();
      recording_ = false;
      ROS_INFO_STREAM("Rosbagger: Rosbag '" << bag_name_ << "' closed.");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't stop recording, since no rosbagger is not in recording mode.");
      return false;
    }
  };

  virtual bool storeMotion(const sensor_msgs::JointState& joint_states)
  {
    if(recording_)
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
      ROS_WARN_STREAM("Rosbagger: Can't write to rosbag, since rosbagger is not in recording mode!");
      return false;
    }
  };

  virtual bool preparePlayback(std::string& filename)
  {
    if (!recording_)
    {
      bag_name_ = filename;
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
          return false;
        }
        view_ptr_ = boost::shared_ptr<rosbag::View>(new rosbag::View(bag_,
                                                                     rosbag::TopicQuery(recorded_motion_topic_name_)));
        view_it_ = view_ptr_->begin();
        ROS_INFO_STREAM("Rosbagger: Ready to read from rosbag '" << bag_name_ << "'.");
        playback_ = true;
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("Rosbagger: Can't open bag, since no filename was provided!");
        throw std::invalid_argument(bag_name_);
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't prepare play back, since rosbagger is in recording mode.");
      return false;
    }
  }

  virtual bool stopPlayback()
  {
    if (playback_)
    {
      playback_ = false;
      bag_.close();
      ROS_INFO_STREAM("Rosbagger: Rosbag '" << bag_name_ << "' closed.");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't stop play back, since rosbagger is in not in playback mode.");
      return false;
    }
  }

  virtual bool readMotion(sensor_msgs::JointState& joint_states)
  {
    if (playback_)
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
      ROS_INFO_STREAM("Rosbagger: We reached the end of the rosbag.");
      return false;
    }
    else
    {
      ROS_WARN_STREAM("Rosbagger: Can't read from rosbag, since rosbagger is not in playback mode!");
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
   * Flag indicating, if the rosbagger is in recording mode
   */
  bool recording_;
  /**
   * Flag indicating, if the rosbagger is in play back mode
   */
  bool playback_;
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
