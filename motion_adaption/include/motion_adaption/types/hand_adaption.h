/**
 * License: BSD
 * https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_adaption/LICENSE
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_ADAPTION_HAND_ADAPTION_H_
#define MOTION_ADAPTION_HAND_ADAPTION_H_

#ifndef DEBUG
#define DEBUG

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include "adaption_type.h"

namespace motion_adaption
{
/**
 *
 */
class HandAdaption : public AdaptionType
{
public:
  HandAdaption(const HandAdaptionParameters& hand_adaption_params,
               boost::shared_ptr<tf::TransformListener> tf_listener,
               boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
               boost::shared_ptr<tf::Transformer> internal_tf) :
               AdaptionType(hand_adaption_params,
                            tf_listener,
                            tf_broadcaster,
                            internal_tf),
               hand_adaption_params_(hand_adaption_params),
               eps(0.01)
  {
    tf_input_stamp_ =  ros::Time::now();
    std::cout << "HandAdaption constructor: adaption name: " << hand_adaption_params_.adaption_name << std::endl;
    tf_hand_scaled_.setIdentity();
    tf_shoulder_scaled_.setIdentity();
    tf_hand_goal_.setIdentity();
  };

  ~HandAdaption(){};

  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    ROS_DEBUG_STREAM("Performing adaption '" << hand_adaption_params_.adaption_name
                    << "' of type 'HandAdaption'.");
    /*
     * Get all input transforms
     */
    if(!getTransforms())
    {
      return false;
    }
    /*
     * Determine the system properties (measurements) of the input and target system
     */
    if(!determineBodyProperties())
    {
      return false;
    }
    /*
     * Use input and target system's proportions to adapt the goal hand positions
     */
    scaleShoulderPosition();

    if (use_elbow_)
    {
      scaleElbowPosition();
    }

    scaleHandPosition();

    /*
     * Align hand orientations with shoulder orientation
     * TODO: Allow the use of extra orientation information (e.g. from IMU sensors)
     */
    if(!adjustHandOrientation())
    {
      return false;
    }
    applyGoalOrientationAdjustments();
    convertTfToPoseMsg(adapted_transforms);
    return true;
  };

private:
  HandAdaptionParameters hand_adaption_params_;
  tf::StampedTransform tf_input_neck_, tf_input_neck_shoulder_, tf_input_shoulder_,tf_input_shoulder_elbow_;
  tf::StampedTransform tf_shoulder_elbow_, tf_shoulder_hand_, tf_input_elbow_, tf_input_elbow_hand_, tf_input_hand_;
  tf::StampedTransform tf_target_neck_, tf_target_neck_shoulder_, tf_target_shoulder_, tf_target_elbow_;
  tf::StampedTransform tf_target_shoulder_elbow_, tf_target_elbow_hand_, tf_target_shoulder_hand_;
  tf::StampedTransform tf_shoulder_scaled_, tf_elbow_scaled_, tf_hand_scaled_, tf_hand_orient_;
  tf::StampedTransform tf_shoulder_goal_, tf_hand_goal_;
  tf::Vector3 vec_helper_, vec_hand_orient_, vec_cross_, vec_cross2_;
  tf::Matrix3x3 mat_orientation_;
  double input_shoulder_width_, input_shoulder_height_, input_shoulder_offset_;
  double input_upper_arm_length_, input_arm_length_;
  double target_shoulder_width_, target_shoulder_height_, target_shoulder_offset_;
  double target_upper_arm_length_, target_lower_arm_length_, target_arm_length_;
  double x_norm_, y_norm_, z_norm_, x_adapt_, y_adapt_, z_adapt_;
  bool use_elbow_;
  ros::Time tf_input_stamp_;
  double eps;

  /**
   *
   * @return
   */
  bool getTransforms()
  {
    // Input transforms
    try
    {
      // reference to neck
      tf_listener_->lookupTransform(hand_adaption_params_.input_ref_name,
                                    hand_adaption_params_.input_neck_name,
                                    ros::Time(0), tf_input_neck_);
      tf_input_stamp_ = tf_input_neck_.stamp_;
      // neck to shoulder
      tf_listener_->lookupTransform(hand_adaption_params_.input_neck_name,
                                   hand_adaption_params_.input_shoulder_name,
                                   ros::Time(0), tf_input_neck_shoulder_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_neck_shoulder_.stamp_);
      // reference to shoulder
      tf_listener_->lookupTransform(hand_adaption_params_.input_ref_name,
                                   hand_adaption_params_.input_shoulder_name,
                                   ros::Time(0), tf_input_shoulder_);
      tf_input_stamp_ = tf_input_shoulder_.stamp_;
      // reference to elbow
      tf_listener_->lookupTransform(hand_adaption_params_.input_ref_name,
                                   hand_adaption_params_.input_elbow_name,
                                   ros::Time(0), tf_input_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_elbow_.stamp_);
      // reference to hand
      tf_listener_->lookupTransform(hand_adaption_params_.input_ref_name,
                                   hand_adaption_params_.input_hand_name,
                                   ros::Time(0), tf_input_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_hand_.stamp_);
      // shoulder to elbow
      tf_listener_->lookupTransform(hand_adaption_params_.input_shoulder_name,
                                   hand_adaption_params_.input_elbow_name,
                                   ros::Time(0), tf_input_shoulder_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_shoulder_elbow_.stamp_);
      // elbow to hand
      tf_listener_->lookupTransform(hand_adaption_params_.input_elbow_name,
                                   hand_adaption_params_.input_hand_name,
                                   ros::Time(0), tf_input_elbow_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_elbow_hand_.stamp_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Hands Adaption: Couldn't get one or more input transformations. Aborting adaption!");
      ROS_DEBUG_STREAM("Hands Adaption: tf error: " << ex.what());
      return false;
    }

    // Target transforms
    try
    {
      // reference to neck
      tf_listener_->lookupTransform(hand_adaption_params_.target_ref_name,
                                    hand_adaption_params_.target_neck_name,
                                    ros::Time(0),
                                    tf_target_neck_);
      // neck to shoulder
      tf_listener_->lookupTransform(hand_adaption_params_.target_neck_name,
                                    hand_adaption_params_.target_shoulder_name,
                                    ros::Time(0),
                                    tf_target_neck_shoulder_);
      // reference to shoulder
      tf_listener_->lookupTransform(hand_adaption_params_.target_ref_name,
                                    hand_adaption_params_.target_shoulder_name,
                                    ros::Time(0),
                                    tf_target_shoulder_);
      if (hand_adaption_params_.target_elbow_name != "")
      {
        // reference to elbow
        tf_listener_->lookupTransform(hand_adaption_params_.target_ref_name,
                                      hand_adaption_params_.target_elbow_name,
                                      ros::Time(0),
                                      tf_target_elbow_);
        // shoulder to elbow
        tf_listener_->lookupTransform(hand_adaption_params_.target_shoulder_name,
                                      hand_adaption_params_.target_elbow_name,
                                      ros::Time(0),
                                      tf_target_shoulder_elbow_);
        // elbow to hand
        tf_listener_->lookupTransform(hand_adaption_params_.target_elbow_name,
                                      hand_adaption_params_.target_hand_name,
                                      ros::Time(0),
                                      tf_target_elbow_hand_);
        use_elbow_ = true;
        ROS_DEBUG_STREAM("Motion Adaption: Will use elbow.");
      }
      else
      {
        // shoulder to hand
        tf_listener_->lookupTransform(hand_adaption_params_.target_shoulder_name,
                                      hand_adaption_params_.target_hand_name,
                                      ros::Time(0),
                                      tf_target_shoulder_hand_);
        use_elbow_ = false;
        ROS_DEBUG_STREAM("Motion Adaption: Won't use elbow.");
      }
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("hand adaption: Couldn't get one or more target transformations. Aborting adaption!");
      ROS_DEBUG_STREAM("hand adaption: tf error: " << ex.what());
      return false;
    }
    return true;
  }

  /**
   * Determine input and target body's properties, such as shoulder width and arm length
   *
   * Assumptions here: x points forward, y to the side, z upwards (right hand system)
   *
   * TODO: Avoid assumptions by finding a smarter way to handle this.
   *
   * @return true, if properties are valid, false otherwise
   */
  bool determineBodyProperties()
  {
    input_shoulder_height_ = std::abs(tf_input_shoulder_.getOrigin().z());
    input_shoulder_width_ = std::abs(tf_input_shoulder_.getOrigin().y()) * 2;
    input_shoulder_offset_ = std::abs(tf_input_shoulder_.getOrigin().x());
    input_upper_arm_length_ = tf_input_shoulder_elbow_.getOrigin().length();
    input_arm_length_ = input_upper_arm_length_ + tf_input_elbow_hand_.getOrigin().length();
    ROS_DEBUG_STREAM("input_shoulder_height  = " << input_shoulder_height_
                     << ", input_shoulder_width  = " << input_shoulder_width_
                     << ", input_shoulder_offset  = " << input_shoulder_offset_
                     << ", input_arm_length  = " << input_arm_length_
                     << ", input_upper_arm_length  = " << input_upper_arm_length_);

    target_shoulder_height_ = std::abs(tf_target_shoulder_.getOrigin().z());
    target_shoulder_width_ = std::abs(tf_target_shoulder_.getOrigin().y()) * 2;
    target_shoulder_offset_ = std::abs(tf_target_shoulder_.getOrigin().x());
    if (use_elbow_)
    {
      // not using the length of the vector, but the biggest scalar in order to avoid wrong arm lengths
      // due to joint/link offsets and similar
      target_upper_arm_length_ = tf_target_shoulder_elbow_.getOrigin()[tf_target_shoulder_elbow_.getOrigin().maxAxis()];
      target_lower_arm_length_ = tf_target_elbow_hand_.getOrigin()[tf_target_elbow_hand_.getOrigin().maxAxis()];
      target_arm_length_ = target_upper_arm_length_ + target_lower_arm_length_;
    }
    else
    {
      target_upper_arm_length_ = 0.0;
      target_lower_arm_length_ = 0.0;
      target_arm_length_ = tf_target_shoulder_hand_.getOrigin().length();
    }
    ROS_DEBUG_STREAM("target_shoulder_height = " << target_shoulder_height_
                     << ", target_shoulder_width = " << target_shoulder_width_
                     << ", target_shoulder_offset = " << target_shoulder_offset_
                     << ", target_arm_length = " << target_arm_length_
                     << ", target_upper_arm_length  = " << target_upper_arm_length_);

    if ((input_upper_arm_length_ > 0.0) && (input_arm_length_ > 0.0) && (target_arm_length_ > 0.0))
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Hand Adaption: One or more proportions of the input and/or target system are not valid. "
                      << "Aborting adaption!");
      return false;
    }
  }

  /**
   * Scaling assumes certain the reference frame being a right hand system with the following properties:
   * x points to the front, y points to the left, z points up
   * @return
   */
  void scaleShoulderPosition()
  {
    if ((input_shoulder_offset_ > eps) && (target_shoulder_offset_ > eps))
    {
      x_norm_ = tf_input_shoulder_.getOrigin().x() / input_shoulder_offset_;
      x_adapt_ = x_norm_ * target_shoulder_offset_;
    }
    else // no scaling - use target shoulder position
    {
      x_adapt_ = x_norm_ = tf_target_shoulder_.getOrigin().x();
    }
    if ((input_shoulder_width_ > eps) && (target_shoulder_width_ > eps))
    {
      y_norm_ = -tf_input_shoulder_.getOrigin().y() / (0.5 * input_shoulder_width_);
      y_adapt_ = y_norm_ * (0.5 * target_shoulder_width_);
    }
    else // no scaling - use target shoulder position
    {
      y_adapt_ = y_norm_ = tf_target_shoulder_.getOrigin().y();
    }
    if ((input_shoulder_height_ > eps) && (target_shoulder_height_ > eps))
    {
      z_norm_ = tf_input_shoulder_.getOrigin().z() / input_shoulder_height_;
      z_adapt_ = z_norm_ * target_shoulder_height_;
    }
    else // no scaling - use target shoulder position
    {
      z_adapt_ = z_norm_ = tf_target_shoulder_.getOrigin().z();
    }

    ROS_DEBUG_STREAM("shoulder - input position:      x = " << tf_input_shoulder_.getOrigin().x()
                     << ", y = " << tf_input_shoulder_.getOrigin().y()
                     << ", z = " << tf_input_shoulder_.getOrigin().z());
    ROS_DEBUG_STREAM("shoulder - normalised position: x = " << x_norm_ << ", y = " << y_norm_
                     << ", z = " << z_norm_);
    ROS_DEBUG_STREAM("shoulder - adapted position:    x = " << x_adapt_ << ", y = " << y_adapt_
                     << ", z = " << z_adapt_);

    tf_shoulder_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));
    internal_tf_->setTransform(tf::StampedTransform(tf_shoulder_scaled_, tf_input_stamp_,
                                                    hand_adaption_params_.target_ref_name, "/shoulder_pos_scaled"));
#ifdef DEBUG
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_shoulder_scaled_, ros::Time::now(),
                                                        hand_adaption_params_.target_ref_name,
                                                        "/shoulder_pos_scaled"));
#endif
  }

  /**
   * Scaling assumes certain the reference frame being a right hand system with the following properties:
   * x points to the front, y points to the left, z points up
   * @return
   */
  void scaleElbowPosition()
  {
    x_norm_ = tf_input_elbow_.getOrigin().x() / input_upper_arm_length_ - input_shoulder_offset_;
    x_adapt_ = x_norm_ * target_upper_arm_length_ + target_shoulder_offset_;
    y_norm_ = (tf_input_elbow_.getOrigin().y() + 0.5 * input_shoulder_width_) / input_upper_arm_length_;
    y_adapt_ = (y_norm_ * target_upper_arm_length_) - 0.5 * target_shoulder_width_;
    z_norm_ = (tf_input_elbow_.getOrigin().z() - input_shoulder_height_)/ input_upper_arm_length_;
    z_adapt_ = (z_norm_ * target_upper_arm_length_) + target_shoulder_height_;

    ROS_DEBUG_STREAM("elbow - input position:      x = " << tf_input_elbow_.getOrigin().x()
                     << ", y = " << tf_input_elbow_.getOrigin().y()
                     << ", z = " << tf_input_elbow_.getOrigin().z());
    ROS_DEBUG_STREAM("elbow - normalised position: x = " << x_norm_ << ", y = " << y_norm_
                     << ", z = " << z_norm_);
    ROS_DEBUG_STREAM("elbow - adapted position:    x = " << x_adapt_ << ", y = " << y_adapt_
                     << ", z = " << z_adapt_);

    tf_elbow_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));
    internal_tf_->setTransform(tf::StampedTransform(tf_elbow_scaled_, tf_input_stamp_,
                                                    hand_adaption_params_.target_ref_name, "/elbow_pos_scaled"));
#ifdef DEBUG
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_elbow_scaled_, ros::Time::now(),
                                                        hand_adaption_params_.target_ref_name,
                                                        "/elbow_pos_scaled"));
#endif
  }

  /**
   * This assumes that the shoulder height is always larger than 0.5 * shoulder width (should be ok, most of the times)
   * TODO: Find a smarter way of handling this
   * @return
   */
  void scaleHandPosition()
  {
    x_norm_ = tf_input_hand_.getOrigin().x() / input_arm_length_ - input_shoulder_offset_;
    x_adapt_ = x_norm_ * target_arm_length_ + target_shoulder_offset_;
    y_norm_ = (tf_input_hand_.getOrigin().y() + 0.5 * input_shoulder_width_) / input_arm_length_;
    y_adapt_ = y_norm_ * target_arm_length_;
    z_norm_ = (tf_input_hand_.getOrigin().z() - input_shoulder_height_)/ input_arm_length_;
    z_adapt_ = (z_norm_ * target_arm_length_) + target_shoulder_height_;

    ROS_DEBUG_STREAM("hand - input position:      x = " << tf_input_hand_.getOrigin().x()
                     << ", y = " << tf_input_hand_.getOrigin().y() << ", z = " << tf_input_hand_.getOrigin().z());
    ROS_DEBUG_STREAM("hand - normalised position: x = " << x_norm_ << ", y = " << y_norm_ << ", z = " << z_norm_);
    ROS_DEBUG_STREAM("hand - adapted position:    x = " << x_adapt_ << ", y = " << y_adapt_ << ", z = " << z_adapt_);

    tf_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));
    internal_tf_->setTransform(tf::StampedTransform(tf_hand_scaled_, tf_input_stamp_,
                                                    hand_adaption_params_.target_ref_name, "/hand_pos_scaled"));
#ifdef DEBUG
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_hand_scaled_, ros::Time::now(),
                                                        hand_adaption_params_.target_ref_name,
                                                        "/hand_pos_scaled"));
#endif
  }

  /**
   * Hand orientation is set to align with the shoulder or - if elbow is used - the elbow orientations
   * For case in which the hand orientation differs from the elbow/shoulder orientation, the manual goal output
   * orientation is applied as well.
   */
  bool adjustHandOrientation()
  {
    tf_hand_goal_.setOrigin(tf_hand_scaled_.getOrigin());
    if (use_elbow_)
    {
      try
      {
        internal_tf_->lookupTransform("/elbow_pos_scaled", "/hand_pos_scaled",
                                      ros::Time(0), tf_hand_orient_);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_WARN_STREAM("hand adaption: Couldn't get scaled elbow to hand transformation! Aborting adaption.");
        ROS_DEBUG_STREAM("hand adaption: tf error: " << ex.what());
        return false;
      }
    }
    else
    {
      try
      {
        internal_tf_->lookupTransform("/shoulder_pos_scaled", "/hand_pos_scaled",
                                      ros::Time(0), tf_hand_orient_);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_WARN_STREAM("hand adaption: Couldn't get scaled shoulder to hand transformation! Aborting adaption.");
        ROS_DEBUG_STREAM("hand adaption: tf error: " << ex.what());
        return false;
      }
    }
    vec_hand_orient_ = tf_hand_orient_.getOrigin();
    vec_hand_orient_.normalize();
    vec_cross_ = tf::Vector3(0.0, 0.0, 1.0).cross(vec_hand_orient_);
    vec_cross_.normalize();
    vec_cross2_ = vec_hand_orient_.cross(vec_cross_);
    vec_cross2_.normalize();
    mat_orientation_.setValue(vec_hand_orient_.x(), vec_cross_.x(), vec_cross2_.x(),
                              vec_hand_orient_.y(), vec_cross_.y(), vec_cross2_.y(),
                              vec_hand_orient_.z(), vec_cross_.z(), vec_cross2_.z());
    tf_hand_goal_.setBasis(mat_orientation_);
    return true;
  }

  /**
   *
   */
  void applyGoalOrientationAdjustments()
  {
    quat_ = tf_hand_goal_.getRotation();
    quat_adapt_.setRPY(hand_adaption_params_.goal_hand_orient_adjust.roll,
                       hand_adaption_params_.goal_hand_orient_adjust.pitch,
                       hand_adaption_params_.goal_hand_orient_adjust.yaw);
    tf_hand_goal_.setRotation(quat_ * quat_adapt_);
  }

  /**
   *
   */
  void convertTfToPoseMsg(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    // Right hand
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hand_adaption_params_.target_ref_name;
    pose_adapted_.pose.position.x = tf_hand_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_hand_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_hand_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_hand_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_hand_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_hand_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_hand_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_hand_goal_, ros::Time::now(),
                                                        hand_adaption_params_.target_ref_name,
                                                        hand_adaption_params_.goal_hand_name));
  }
};

} // namespace motion_adaption

#endif /* DEBUG */
#endif /* MOTION_ADAPTION_HAND_ADAPTION_H_ */
