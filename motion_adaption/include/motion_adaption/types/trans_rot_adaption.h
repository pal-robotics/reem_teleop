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
  TransRotAdaption(const TransRotAdaptionParameters& trans_rot_adapt_params,
                     boost::shared_ptr<tf::TransformListener> tf_listener,
                     boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                     boost::shared_ptr<tf::Transformer> internal_tf) :
                     AdaptionType(trans_rot_adapt_params,
                                  tf_listener,
                                  tf_broadcaster,
                                  internal_tf),
                     trans_rot_adapt_params_(trans_rot_adapt_params)
  {
    std::cout << "TransRotAdaption constructor: adaption name: " << trans_rot_adapt_params_.adaption_name << std::endl;
    std::cout << "TransRotAdaption constructor: input_endpt_name: " << trans_rot_adapt_params_.input_endpt_name << std::endl;
  };

  ~TransRotAdaption(){};

  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    ROS_DEBUG_STREAM("Performing adaption '" << trans_rot_adapt_params_.adaption_name
                    << "' of type 'TransRotAdaption'.");
    /*
     * Set the reference frame
     */
    setReferenceFrame();
    /*
     * Get the input motion
     */
    try
    {
      tf_listener_->lookupTransform(trans_rot_adapt_params_.input_ref_name,
                                    trans_rot_adapt_params_.input_endpt_name,
                                    ros::Time(0),
                                    tf_input_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Coulnd't get transform from '" << trans_rot_adapt_params_.input_ref_name
                      << "' to '" << trans_rot_adapt_params_.input_endpt_name
                      << "'! Skipping trans rot adaption.");
      ROS_DEBUG_STREAM(ex.what());
      return false;
    }
    /*
     * Get target's transforms for adaption
     */
    try
    {
      tf_listener_->lookupTransform(trans_rot_adapt_params_.target_ref_name,
                                    trans_rot_adapt_params_.target_endpt_name,
                                    ros::Time(0),
                                    tf_target_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Coulnd't get transform from '" << trans_rot_adapt_params_.target_ref_name
                      << "' to '" << trans_rot_adapt_params_.target_endpt_name
                      << "'! Skipping trans rot adaption.");
      ROS_DEBUG_STREAM(ex.what());
      return false;
    }
    /*
     * Adapt the input motion
     */
    tf_adapted_.stamp_ = tf_input_.stamp_;
    tf_adapted_.frame_id_ = tf_target_.frame_id_;
    tf_adapted_.child_frame_id_ = tf_target_.child_frame_id_;
    // Scale the translation
    vec_ = tf_target_.getOrigin(); // no scaling
    tf_adapted_.setOrigin(vec_);
    /*
     * Apply the output orientation adjustment to align with the target orientation
     * Probably can be done automatically
     */
    quat_ = tf_input_.getRotation();
    quat_adapt_.setRPY(trans_rot_adapt_params_.goal_orient_adjust.roll,
                       trans_rot_adapt_params_.goal_orient_adjust.pitch,
                       trans_rot_adapt_params_.goal_orient_adjust.yaw);
    tf_adapted_.setRotation(quat_ * quat_adapt_);

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
                                                        trans_rot_adapt_params_.target_ref_name,
                                                        trans_rot_adapt_params_.goal_endpt_name));
    return true;
  };

private:
  TransRotAdaptionParameters trans_rot_adapt_params_;
};

} // namespace motion_adaption

#endif /* TRANS_ROT_ADAPTION_H_ */

