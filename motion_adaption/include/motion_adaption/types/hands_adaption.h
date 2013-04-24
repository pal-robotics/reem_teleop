/**
 * @file /motion_adaption/include/motion_adaption/types/hands_adaption.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 17, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HANDS_ADAPTION_H_
#define HANDS_ADAPTION_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "adaption_type.h"

namespace motion_adaption
{

class HandsAdaption : public AdaptionType
{
public:
  HandsAdaption(const HandsAdaptionParameters& hands_adaption_params,
                  boost::shared_ptr<tf::TransformListener> tf_listener,
                  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                  boost::shared_ptr<tf::Transformer> internal_tf) :
                  AdaptionType(hands_adaption_params,
                               tf_listener,
                               tf_broadcaster,
                               internal_tf),
                  hands_adaption_params_(hands_adaption_params)
  {
    tf_input_stamp_ =  ros::Time::now();
    std::cout << "HandsAdaption constructor: adaption name: " << hands_adaption_params_.adaption_name << std::endl;
    std::cout << "HandsAdaption constructor: input_endpt_name: " << hands_adaption_params_.input_l_elbow_name << std::endl;
    tf_r_hand_scaled_.setIdentity();
    tf_l_hand_scaled_.setIdentity();
    tf_r_shoulder_scaled_.setIdentity();
    tf_l_shoulder_scaled_.setIdentity();
    tf_r_hand_goal_.setIdentity();
    tf_l_hand_goal_.setIdentity();
  };

  ~HandsAdaption(){};

  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    ROS_DEBUG_STREAM("Performing adaption '" << hands_adaption_params_.adaption_name
                    << "' of type 'HandsAdaption'.");
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
    if(!determineSystemProperties())
    {
      return false;
    }
    /*
     * Use input and target system's proportions to adapt the goal hand positions
     */
    scaleHandPositions();

    if (use_elbows_)
    {
      ROS_ERROR_STREAM("Using elbows hasn't been fully implemented yet! Aborting adaption.");
      return false;
    }

    scaleShoulderPositions();
    /*
     * Use the adapted hand positions to determine the shoulder orientation
     */
//    if(!adaptShoulderOrientations())
//    {
//      return false;
//    }
//    /*
//     * Align hand orientations with shoulder orientation
//     * TODO: Allow the use of extra orientation information (e.g. from IMU sensors)
//     */
//    if(!adaptHandOrientations())
//    {
//      return false;
//    }
    if(!adjustHandOrientations())
    {
      return false;
    }
    applyGoalOrientationAdjustments();
    convertTfToPoseMsg(adapted_transforms);
    return true;
  };

private:
  HandsAdaptionParameters hands_adaption_params_;
  tf::StampedTransform tf_input_r_shoulder_,tf_input_r_shoulder_elbow_, tf_r_shoulder_elbow_, tf_r_shoulder_hand_, tf_input_r_elbow_, tf_input_r_elbow_hand_, tf_input_r_hand_;
  tf::StampedTransform tf_input_l_shoulder_,tf_input_l_shoulder_elbow_, tf_l_shoulder_elbow_, tf_l_shoulder_hand_, tf_input_l_elbow_, tf_input_l_elbow_hand_, tf_input_l_hand_, tf_input_r_shoulder_l_shoulder_;
  tf::StampedTransform tf_target_r_shoulder_, tf_target_r_elbow_, tf_target_r_shoulder_r_elbow_, tf_target_r_elbow_r_hand_;
  tf::StampedTransform tf_target_r_shoulder_r_hand_, tf_target_r_shoulder_l_shoulder_;
  tf::StampedTransform tf_target_l_shoulder_, tf_target_l_elbow_;
  tf::StampedTransform tf_r_shoulder_scaled_, tf_r_elbow_scaled_, tf_r_hand_scaled_;
  tf::StampedTransform tf_l_shoulder_scaled_, tf_l_elbow_scaled_, tf_l_hand_scaled_;
  tf::StampedTransform tf_r_hand_orient_, tf_l_hand_orient_;
  tf::StampedTransform tf_r_shoulder_goal_, tf_r_hand_goal_, tf_l_shoulder_goal_, tf_l_hand_goal_;
  tf::Vector3 vec_helper_, vec_hand_orient_, vec_cross_, vec_cross2_;
  tf::Matrix3x3 mat_orientation_;
  double input_shoulder_width_, input_shoulder_height_, input_upper_arm_length_, input_arm_length_;
  double target_shoulder_width_, target_shoulder_height_, target_upper_arm_length_, target_lower_arm_length_, target_arm_length_;
  double x_norm_, y_norm_, z_norm_, x_adapt_, y_adapt_, z_adapt_;
  bool use_elbows_;
  ros::Time tf_input_stamp_;

  /**
   *
   * @return
   */
  bool getTransforms()
  {
    // Input transforms
    try
    {
      // reference to right shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_shoulder_name,
                                   ros::Time(0), tf_input_r_shoulder_);
      tf_input_stamp_ = tf_input_r_shoulder_.stamp_;
      // reference to right elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_elbow_name,
                                   ros::Time(0), tf_input_r_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_r_elbow_.stamp_);
      // reference to right hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_hand_name,
                                   ros::Time(0), tf_input_r_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_r_hand_.stamp_);
      // right shoulder to right elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_shoulder_name,
                                   hands_adaption_params_.input_r_elbow_name,
                                   ros::Time(0), tf_input_r_shoulder_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_r_shoulder_elbow_.stamp_);
      // right elbow to right hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_elbow_name,
                                   hands_adaption_params_.input_r_hand_name,
                                   ros::Time(0), tf_input_r_elbow_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_r_elbow_hand_.stamp_);
      // reference to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                    hands_adaption_params_.input_l_shoulder_name,
                                    ros::Time(0), tf_input_l_shoulder_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_l_shoulder_.stamp_);
      // reference to left elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_l_elbow_name,
                                   ros::Time(0), tf_input_l_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_l_elbow_.stamp_);
      // reference to left hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_l_hand_name,
                                   ros::Time(0), tf_input_l_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_l_hand_.stamp_);
      // left shoulder to left elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_l_shoulder_name,
                                   hands_adaption_params_.input_l_elbow_name,
                                   ros::Time(0), tf_input_l_shoulder_elbow_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_l_shoulder_elbow_.stamp_);
      // left elbow to left hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_l_elbow_name,
                                   hands_adaption_params_.input_l_hand_name,
                                   ros::Time(0), tf_input_l_elbow_hand_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_l_elbow_hand_.stamp_);
      // right shoulder to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_shoulder_name,
                                   hands_adaption_params_.input_l_shoulder_name,
                                   ros::Time(0), tf_input_r_shoulder_l_shoulder_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_input_r_shoulder_l_shoulder_.stamp_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("hands adaption: Couldn't get one or more input transformations. Aborting adaption!");
      ROS_DEBUG_STREAM("hands adaption: tf error: " << ex.what());
      return false;
    }

    // Target transforms
    try
    {
      // reference to right shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.target_ref_name,
                                    hands_adaption_params_.target_r_shoulder_name,
                                    ros::Time(0),
                                    tf_target_r_shoulder_);
      // reference to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.target_ref_name,
                                    hands_adaption_params_.target_l_shoulder_name,
                                    ros::Time(0),
                                    tf_target_r_shoulder_);
      // right to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                    hands_adaption_params_.target_l_shoulder_name,
                                    ros::Time(0),
                                    tf_target_r_shoulder_l_shoulder_);
      if ((hands_adaption_params_.target_r_elbow_name != "")
          && (hands_adaption_params_.target_l_elbow_name != ""))
      {
        // reference to right elbow
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                      hands_adaption_params_.target_r_elbow_name,
                                      ros::Time(0),
                                      tf_target_r_elbow_);
        // reference to right elbow
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                      hands_adaption_params_.target_l_elbow_name,
                                      ros::Time(0),
                                      tf_target_l_elbow_);
        // right shoulder to right elbow
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                      hands_adaption_params_.target_r_elbow_name,
                                      ros::Time(0),
                                      tf_target_r_shoulder_r_elbow_);
        // right elbow to right hand
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_elbow_name,
                                      hands_adaption_params_.target_r_hand_name,
                                      ros::Time(0),
                                      tf_target_r_elbow_r_hand_);
        use_elbows_ = true;
        ROS_DEBUG_STREAM("Motion Adaption: Will use elbows.");
      }
      else
      {
        // right shoulder to right hand
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                      hands_adaption_params_.target_r_hand_name,
                                      ros::Time(0),
                                      tf_target_r_shoulder_r_hand_);
        use_elbows_ = false;
        ROS_DEBUG_STREAM("Motion Adaption: Won't use elbows.");
      }
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("hands adaption: Couldn't get one or more target transformations. Aborting adaption!");
      ROS_DEBUG_STREAM("hands adaption: tf error: " << ex.what());
      return false;
    }
    return true;
  }

  /**
   * This assumes that shoulder height is always larger than 0.5 * shoulder width (should be ok, most of the times)
   * TODO: Find a smarter way of handling this
   * @return
   */
  bool determineSystemProperties()
  {
    vec_helper_ = (tf_input_r_shoulder_.getOrigin() + tf_input_l_shoulder_.getOrigin()) / 2.0;
    input_shoulder_height_ = sqrt(pow(vec_helper_[vec_helper_.absolute().maxAxis()], 2));
    input_shoulder_width_ = tf_input_r_shoulder_l_shoulder_.getOrigin().length();
    input_upper_arm_length_ = tf_input_r_shoulder_elbow_.getOrigin().length();
    input_arm_length_ = tf_input_r_shoulder_elbow_.getOrigin().length() + tf_input_r_elbow_hand_.getOrigin().length();
//    ROS_DEBUG_STREAM_THROTTLE(1.0, "input_shoulder_height = " << input_shoulder_height_
//                                   << ", input_shoulder_width = " << input_shoulder_width_
//                                   << ", input_arm_length = " << input_arm_length_);
    ROS_DEBUG_STREAM("input_shoulder_height = " << input_shoulder_height_
                     << ", input_shoulder_width = " << input_shoulder_width_
                     << ", input_arm_length = " << input_arm_length_);


    vec_helper_ = (tf_target_r_shoulder_.getOrigin() + tf_target_r_shoulder_.getOrigin()) / 2.0;
    target_shoulder_height_ = sqrt(pow(vec_helper_[vec_helper_.absolute().maxAxis()], 2));
    target_shoulder_width_ = tf_target_r_shoulder_l_shoulder_.getOrigin().length();
    if (use_elbows_)
    {
      target_upper_arm_length_ = tf_target_r_shoulder_r_elbow_.getOrigin().length();
      target_lower_arm_length_ = tf_target_r_elbow_r_hand_.getOrigin().length();
      target_arm_length_ = target_upper_arm_length_ + target_lower_arm_length_;
    }
    else
    {
      target_arm_length_ = tf_target_r_shoulder_r_hand_.getOrigin().length();
    }
//    ROS_DEBUG_STREAM_THROTTLE(1.0, "target_shoulder_height = " << target_shoulder_height_
//                                   << ", target_shoulder_width = " << target_shoulder_width_
//                                   << ", target_arm_length = " << target_arm_length_);
    ROS_DEBUG_STREAM("target_shoulder_height = " << target_shoulder_height_
                     << ", target_shoulder_width = " << target_shoulder_width_
                     << ", target_arm_length = " << target_arm_length_);

    if ((input_shoulder_width_ > 0.0) && (input_shoulder_height_ > 0.0) && (input_upper_arm_length_ > 0.0)
        && (input_arm_length_ > 0.0) && (target_shoulder_height_ > 0.0) && (target_shoulder_width_ > 0.0)
        && (target_arm_length_ > 0.0))
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Hands Adaption: One or more proportions of the input and/or target system are not valid. "
                      << "Aborting adaption!");
      return false;
    }
  }

  /**
   * This assumes that shoulder height is always larger than 0.5 * shoulder width (should be ok, most of the times)
   * TODO: Find a smarter way of handling this
   * @return
   */
  void scaleHandPositions()
  {
    ROS_DEBUG_STREAM("right hand - input position: " << tf_input_r_hand_.getOrigin().x()
                     << ", " << tf_input_r_hand_.getOrigin().y() << ", " << tf_input_r_hand_.getOrigin().z());
//      x_norm_ = tf_input_r_hand_.getOrigin().x() / (input_arm_length_ + 0.5 * input_shoulder_width_);
//      y_norm_ = tf_input_r_hand_.getOrigin().y() / (input_arm_length_ + input_shoulder_height_);
//      z_norm_ = tf_input_r_hand_.getOrigin().z() / input_arm_length_;
    x_norm_ = tf_input_r_hand_.getOrigin().x() / input_arm_length_;
    y_norm_ = (tf_input_r_hand_.getOrigin().y() - 0.5 * input_shoulder_width_) / input_arm_length_;
    z_norm_ = (tf_input_r_hand_.getOrigin().z() - input_shoulder_height_)/ input_arm_length_;
    ROS_DEBUG_STREAM("right hand - normalised position: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
//      x_adapt_ = x_norm_ * ( target_arm_length_ + 0.5 * target_shoulder_width_);
//      y_adapt_ = y_norm_ * ( target_arm_length_ + target_shoulder_height_);
//      z_adapt_ = z_norm_ * target_arm_length_;
    x_adapt_ = x_norm_ * target_arm_length_;
    y_adapt_ = (y_norm_ * target_arm_length_) +  0.5 * target_shoulder_width_;
    z_adapt_ = (z_norm_ * target_arm_length_) + target_shoulder_height_;
    ROS_DEBUG_STREAM("right hand - adapted position: " << x_adapt_ << ", " << y_adapt_ << ", " << z_adapt_);
    tf_r_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

//      x_norm_ = tf_input_r_hand_.getOrigin().x() / (input_arm_length_ + 0.5 * input_shoulder_width_);
//      y_norm_ = tf_input_r_hand_.getOrigin().y() / (input_arm_length_ + input_shoulder_height_);
//      z_norm_ = tf_input_r_hand_.getOrigin().z() / input_arm_length_;
    ROS_DEBUG_STREAM("left hand - input position: " << tf_input_l_hand_.getOrigin().x()
                     << ", " << tf_input_r_hand_.getOrigin().y() << ", " << tf_input_r_hand_.getOrigin().z());
    x_norm_ = tf_input_l_hand_.getOrigin().x() / input_arm_length_;
    y_norm_ = (tf_input_l_hand_.getOrigin().y() + 0.5 * input_shoulder_width_) / input_arm_length_;
    z_norm_ = (tf_input_l_hand_.getOrigin().z() - input_shoulder_height_)/ input_arm_length_;
    ROS_DEBUG_STREAM("left hand - normalised position: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
//      x_adapt_ = x_norm_ * ( target_arm_length_ + 0.5 * target_shoulder_width_);
//      y_adapt_ = y_norm_ * ( target_arm_length_ + target_shoulder_height_);
//      z_adapt_ = z_norm_ * target_arm_length_;
    x_adapt_ = x_norm_ * target_arm_length_;
    y_adapt_ = (y_norm_ * target_arm_length_) -  0.5 * target_shoulder_width_;
    z_adapt_ = (z_norm_ * target_arm_length_) + target_shoulder_height_;
    ROS_DEBUG_STREAM("left hand - adapted position: " << x_adapt_ << ", " << y_adapt_ << ", " << z_adapt_);
    tf_l_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

//    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_r_hand_scaled_, tf_input_stamp_,
//                                                        hands_adaption_params_.target_ref_name, "/hand_r_pos_scaled"));
    internal_tf_->setTransform(tf::StampedTransform(tf_r_hand_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/hand_r_pos_scaled"));
//    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_l_hand_scaled_, tf_input_stamp_,
//                                                       hands_adaption_params_.target_ref_name, "/hand_l_pos_scaled"));
    internal_tf_->setTransform(tf::StampedTransform(tf_l_hand_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/hand_l_pos_scaled"));
  }

  /**
   *
   * @return
   */
  void scaleShoulderPositions()
  {
    ROS_DEBUG_STREAM("right shoulder - input position: " << tf_input_r_shoulder_.getOrigin().x()
                     << ", " << tf_input_r_shoulder_.getOrigin().y() << ", " << tf_input_r_shoulder_.getOrigin().z());
    x_norm_ = tf_input_r_shoulder_.getOrigin().x() / (0.5 * input_shoulder_width_);
    y_norm_ = tf_input_r_shoulder_.getOrigin().y() / (0.5 * input_shoulder_width_);
    z_norm_ = tf_input_r_shoulder_.getOrigin().z() / input_shoulder_height_;
    ROS_DEBUG_STREAM("right shoulder - normalised position: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
    x_adapt_ = x_norm_ * (0.5 * target_shoulder_width_);
    y_adapt_ = y_norm_ * (0.5 * target_shoulder_width_);
    z_adapt_ = z_norm_ * target_shoulder_height_;
    ROS_DEBUG_STREAM("right shoulder - adapted position: " << x_adapt_ << ", " << y_adapt_ << ", " << z_adapt_);
    tf_r_shoulder_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

    ROS_DEBUG_STREAM("left shoulder - input position: " << tf_input_l_shoulder_.getOrigin().x()
                     << ", " << tf_input_l_shoulder_.getOrigin().y() << ", " << tf_input_l_shoulder_.getOrigin().z());
    x_norm_ = tf_input_l_shoulder_.getOrigin().x() / (0.5 * input_shoulder_width_);
    y_norm_ = tf_input_l_shoulder_.getOrigin().y() / (0.5 * input_shoulder_width_);
    z_norm_ = tf_input_l_shoulder_.getOrigin().z() / input_shoulder_height_;
    ROS_DEBUG_STREAM("left shoulder - normalised position: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
    x_adapt_ = x_norm_ * (0.5 * target_shoulder_width_);
    y_adapt_ = y_norm_ * (0.5 * target_shoulder_width_);
    z_adapt_ = z_norm_ * target_shoulder_height_;
    ROS_DEBUG_STREAM("left shoulder - adapted position: " << x_adapt_ << ", " << y_adapt_ << ", " << z_adapt_);
    tf_l_shoulder_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, tf_input_stamp_,
                                                        hands_adaption_params_.target_ref_name, "/shoulder_r_pos_scaled"));
    internal_tf_->setTransform(tf::StampedTransform(tf_r_shoulder_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/shoulder_r_pos_scaled"));
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, tf_input_stamp_,
                                                       hands_adaption_params_.target_ref_name, "/shoulder_l_pos_scaled"));
    internal_tf_->setTransform(tf::StampedTransform(tf_l_shoulder_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/shoulder_l_pos_scaled"));
  }

  /**
   * Hand orientations are set to align with the shoulder or - if elbows are used - the elbow orientations
   * For case in which the hand orientations differ from the elbow/shoulder orientations, the manual goal output
   * orientation is applied as well.
   */
  bool adjustHandOrientations()
  {
    tf_r_hand_goal_.setOrigin(tf_r_hand_scaled_.getOrigin());
    tf_l_hand_goal_.setOrigin(tf_l_hand_scaled_.getOrigin());
    if (use_elbows_)
    {
      try
      {
        internal_tf_->lookupTransform("/elbow_r_pos_scaled", "/hands_r_pos_scaled",
                                      ros::Time(0), tf_r_hand_orient_);
        internal_tf_->lookupTransform("/elbowr_l_pos_scaled", "/hands_l_pos_scaled",
                                      ros::Time(0), tf_l_hand_orient_);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_WARN_STREAM("hands adaption: Couldn't get scaled elbow transformations! Aborting adaption.");
        ROS_DEBUG_STREAM("hands adaption: tf error: " << ex.what());
        return false;
      }
    }
    else
    {
      try
      {
        internal_tf_->lookupTransform("/shoulder_r_pos_scaled", "/hand_r_pos_scaled",
                                      ros::Time(0), tf_r_hand_orient_);
        internal_tf_->lookupTransform("/shoulder_l_pos_scaled", "/hand_l_pos_scaled",
                                      ros::Time(0), tf_l_hand_orient_);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_WARN_STREAM("hands adaption: Couldn't get scaled shoulder transformations! Aborting adaption.");
        ROS_DEBUG_STREAM("hands adaption: tf error: " << ex.what());
        return false;
      }
    }
    vec_hand_orient_ = tf_r_hand_orient_.getOrigin();
    vec_hand_orient_.normalize();
    vec_cross_ = tf::Vector3(0.0, 0.0, 1.0).cross(vec_hand_orient_);
    vec_cross_.normalize();
    vec_cross2_ = vec_hand_orient_.cross(vec_cross_);
    vec_cross2_.normalize();
    mat_orientation_.setValue(vec_hand_orient_.x(), vec_cross_.x(), vec_cross2_.x(),
                              vec_hand_orient_.y(), vec_cross_.y(), vec_cross2_.y(),
                              vec_hand_orient_.z(), vec_cross_.z(), vec_cross2_.z());
    tf_r_hand_goal_.setBasis(mat_orientation_);

//    internal_tf_->setTransform(tf::StampedTransform(tf_r_hand_goal_, tf_input_stamp_,
//                                                    hands_adaption_params_.target_ref_name,
//                                                    hands_adaption_params_.goal_r_hand_name));
    vec_hand_orient_ = tf_l_hand_orient_.getOrigin();
    vec_hand_orient_.normalize();
    vec_cross_ = tf::Vector3(0.0, 0.0, 1.0).cross(vec_hand_orient_);
    vec_cross_.normalize();
    vec_cross2_ = vec_hand_orient_.cross(vec_cross_);
    vec_cross2_.normalize();
    mat_orientation_.setValue(vec_hand_orient_.x(), vec_cross_.x(), vec_cross2_.x(),
                              vec_hand_orient_.y(), vec_cross_.y(), vec_cross2_.y(),
                              vec_hand_orient_.z(), vec_cross_.z(), vec_cross2_.z());
    tf_l_hand_goal_.setBasis(mat_orientation_);
//    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(),
//                                                        hands_adaption_params_.target_ref_name,
//                                                        hands_adaption_params_.goal_l_hand_name));
//    internal_tf_->setTransform(tf::StampedTransform(tf_l_hand_goal_, tf_input_stamp_,
//                                                    hands_adaption_params_.target_ref_name,
//                                                    hands_adaption_params_.goal_l_hand_name));
    return true;
  }

  /**
   *
   */
  void applyGoalOrientationAdjustments()
  {
    quat_ = tf_r_hand_goal_.getRotation();
    quat_adapt_.setRPY(hands_adaption_params_.goal_r_hand_orient_adjust.roll,
                       hands_adaption_params_.goal_r_hand_orient_adjust.pitch,
                       hands_adaption_params_.goal_r_hand_orient_adjust.yaw);
    tf_r_hand_goal_.setRotation(quat_ * quat_adapt_);
    quat_ = tf_l_hand_goal_.getRotation();
    quat_adapt_.setRPY(hands_adaption_params_.goal_l_hand_orient_adjust.roll,
                       hands_adaption_params_.goal_l_hand_orient_adjust.pitch,
                       hands_adaption_params_.goal_l_hand_orient_adjust.yaw);
    tf_l_hand_goal_.setRotation(quat_ * quat_adapt_);
  }

  /**
   *
   */
  void convertTfToPoseMsg(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    // Right hand
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.target_ref_name;
    pose_adapted_.pose.position.x = tf_r_hand_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_r_hand_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_r_hand_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_r_hand_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_r_hand_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_r_hand_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_r_hand_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_r_hand_goal_, ros::Time::now(),
                                                        hands_adaption_params_.target_ref_name,
                                                        hands_adaption_params_.goal_r_hand_name));

    // Left hand
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.target_ref_name;
    pose_adapted_.pose.position.x = tf_l_hand_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_l_hand_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_l_hand_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_l_hand_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_l_hand_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_l_hand_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_l_hand_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(),
                                                        hands_adaption_params_.target_ref_name,
                                                        hands_adaption_params_.goal_l_hand_name));
  }
};

} // namespace motion_adaption

#endif /* HANDS_ADAPTION_H_ */
