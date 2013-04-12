/**
 * @file /motion_adaption/include/motion_adaption/types/trans_rot_adaption.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 9, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TRANS_ROT_ADAPTION_H_
#define TRANS_ROT_ADAPTION_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "adaption_type.h"

namespace motion_adaption
{

class TransRotAdaption : public AdaptionType
{
public:
  TransRotAdaption(const MotionAdaptionParameters& adaption_parameters,
                     boost::shared_ptr<tf::TransformListener> tf_listener,
                     boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                     boost::shared_ptr<tf::Transformer> internal_tf) :
                     AdaptionType(adaption_parameters,
                                  tf_listener,
                                  tf_broadcaster,
                                  internal_tf){};
  ~TransRotAdaption(){};

  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    ROS_DEBUG_STREAM("Performing adaption '" << adaption_parameters_.adaption_name
                    << "' of type '" << adaption_parameters_.adaption_type << "'.");
    /*
     * Get the input motion
     */
    try
    {
      tf_listener_->waitForTransform(adaption_parameters_.input_ref_frame,
                                     adaption_parameters_.input_endpts[0],
                                     ros::Time(0),
                                     ros::Duration(wait_for_tf_));
      tf_listener_->lookupTransform(adaption_parameters_.input_ref_frame,
                                    adaption_parameters_.input_endpts[0],
                                    ros::Time(0),
                                    tf_input_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Coulnd't get transform from '" << adaption_parameters_.input_ref_frame
                      << "' to '" << adaption_parameters_.input_endpts[0]
                      << "'! Skipping motion adaption.");
      ROS_DEBUG_STREAM(ex.what());
      return false;
    }
    /*
     * Get target's transforms for adaption
     */
    try
    {
      tf_listener_->waitForTransform(adaption_parameters_.target_ref_frame,
                                     adaption_parameters_.target_endpts[0],
                                     ros::Time(0),
                                     ros::Duration(wait_for_tf_));
      tf_listener_->lookupTransform(adaption_parameters_.target_ref_frame,
                                    adaption_parameters_.target_endpts[0],
                                    ros::Time(0),
                                    tf_target_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Coulnd't get transform from '" << adaption_parameters_.target_ref_frame
                      << "' to '" << adaption_parameters_.target_endpts[0]
                      << "'! Skipping motion adaption.");
      ROS_DEBUG_STREAM(ex.what());
      return false;
    }
    /*
     * Apply the input correction (TODO)
     */
//    quat_ = tf_input_.getRotation();
//    quat_adapt_.setRPY(adaption_parameters_.input_correction.roll,
//                       adaption_parameters_.input_correction.pitch,
//                       adaption_parameters_.input_correction.yaw);
//    tf_input_.setRotation(quat_ * quat_adapt_);
//    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_input_,
//                                                        ros::Time::now(),
//                                                        adaption_parameters_.target_ref_frame,
//                                                        "input_corrected"));
    /*
     * Adapt the input motion
     */
    // Scaling the translation
    vec_ = tf_target_.getOrigin(); // no scaling (scaling factor is 1)
    tf_adapted_.setOrigin(vec_);
    // Scaling the rotation
    quat_ = tf_input_.getRotation(); // no scaling
    tf_adapted_.setRotation(quat_);

    //    internal_tf_.setTransform(tf::StampedTransform(tf_adapted_,
//                                                   tf_calc_time_,
//                                                   "/head_adapted",
//                                                   "/head_goal"));
//    internal_tf_.lookupTransform("/ref_frame",
//                                 "/head_goal", ros::Time(0), tf_adapted_);
//    tf_adapted_ = tf_input_;

     /*
      * Store the result and optionally publish it
      * TODO: Using StampedTransform.msg would make extra endpoint specification obsolete
      */
    pose_adapted_.header.stamp = tf_adapted_.stamp_;
    pose_adapted_.header.frame_id = tf_adapted_.frame_id_;
    pose_adapted_.pose.position.x = tf_adapted_.getOrigin()[0];
    pose_adapted_.pose.position.y = tf_adapted_.getOrigin()[1];
    pose_adapted_.pose.position.z = tf_adapted_.getOrigin()[2];
    pose_adapted_.pose.orientation.x = tf_adapted_.getRotation()[0];
    pose_adapted_.pose.orientation.y = tf_adapted_.getRotation()[1];
    pose_adapted_.pose.orientation.z = tf_adapted_.getRotation()[2];
    pose_adapted_.pose.orientation.w = tf_adapted_.getRotation()[3];
    adapted_transforms.push_back(pose_adapted_);
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_adapted_,
                                                        ros::Time::now(),
                                                        adaption_parameters_.target_ref_frame,
                                                        adaption_parameters_.goal_endpts[0]));
    return true;
  };

private:
};

} // namespace motion_adaption

#endif /* TRANS_ROT_ADAPTION_H_ */

