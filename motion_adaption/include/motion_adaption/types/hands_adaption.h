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
     * Use input and target system's proportions to adapt the goal hand positions
     */
    if(!adaptHandPositions())
    {
      return false;
    }
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
    convertTfToPoseMsg(adapted_transforms);
    return true;
  };

private:
  HandsAdaptionParameters hands_adaption_params_;
  tf::StampedTransform tf_input_torso_;
  tf::StampedTransform tf_input_r_shoulder_,tf_input_r_shoulder_elbow_, tf_r_shoulder_elbow_, tf_r_shoulder_hand_, tf_input_r_elbow_, tf_input_r_elbow_hand_, tf_input_r_hand_;
  tf::StampedTransform tf_input_l_shoulder_,tf_input_l_shoulder_elbow_, tf_l_shoulder_elbow_, tf_l_shoulder_hand_, tf_input_l_elbow_, tf_input_l_elbow_hand_, tf_input_l_hand_, tf_input_r_shoulder_l_shoulder_;
  tf::StampedTransform tf_target_torso_r_shoulder_, tf_target_r_shoulder_r_elbow_, tf_target_r_elbow_r_hand_;
  tf::StampedTransform tf_target_torso_l_shoulder_, tf_target_r_shoulder_r_hand_, tf_target_r_shoulder_l_shoulder_;
  tf::StampedTransform tf_r_shoulder_scaled_, tf_r_elbow_scaled_, tf_r_hand_scaled_;
  tf::StampedTransform tf_l_shoulder_scaled_, tf_l_elbow_scaled_, tf_l_hand_scaled_;
  tf::StampedTransform tf_r_elbow_pos_, tf_r_elbow_orient_, tf_r_elbow_hand_;
  tf::StampedTransform tf_l_elbow_pos_, tf_l_elbow_orient_, tf_l_elbow_hand_;
  tf::StampedTransform tf_r_hand_adjusted_, tf_l_hand_adjusted_;
  tf::StampedTransform tf_r_shoulder_goal_, tf_r_elbow_goal_, tf_r_hand_goal_;
  tf::StampedTransform tf_l_shoulder_goal_, tf_l_elbow_goal_, tf_l_hand_goal_;
  tf::Vector3 vec_shoulder_elbow_, vec_shoulder_hand_, vec_elbow_hand_, vec_normal_, vec_helper_;
  /////////////////// THEY ARE USED BEFORE INITIALISED ... doesn't make sense!
  tf::Vector3 vec_r_elbow_hand_, vec_l_elbow_hand_; // TODO: those might be obsolete, check!
  ///////////////////
  tf::Matrix3x3 mat_orientation_;
  double input_shoulder_width_, input_shoulder_height_, input_upper_arm_length_, input_arm_length_;
  double target_shoulder_width_, target_shoulder_height_, target_upper_arm_length_, target_lower_arm_length_, target_arm_length_;
  double x_norm_, y_norm_, z_norm_, x_adapt_, y_adapt_, z_adapt_, elbow_x_, elbow_y_, limb_length_;
  bool use_elbows_, r_elbow_extended_, l_elbow_extended_;
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
      // input reference to torso - used?
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_torso_name,
                                   ros::Time(0), tf_input_torso_);
      tf_input_stamp_ = tf_input_torso_.stamp_;
      // torso to right shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_shoulder_name,
                                   ros::Time(0), tf_input_r_shoulder_);
      // torso to right elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_elbow_name,
                                   ros::Time(0), tf_input_r_elbow_);
      // torso to right hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_r_hand_name,
                                   ros::Time(0), tf_input_r_hand_);
      // right shoulder to right elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_shoulder_name,
                                   hands_adaption_params_.input_r_elbow_name,
                                   ros::Time(0), tf_input_r_shoulder_elbow_);
      // right elbow to right hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_elbow_name,
                                   hands_adaption_params_.input_r_hand_name,
                                   ros::Time(0), tf_input_r_elbow_hand_);
      // torso to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                    hands_adaption_params_.input_l_shoulder_name,
                                    ros::Time(0), tf_input_l_shoulder_);
      // torso to left elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_l_elbow_name,
                                   ros::Time(0), tf_input_l_elbow_);
      // torso to left hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_ref_name,
                                   hands_adaption_params_.input_l_hand_name,
                                   ros::Time(0), tf_input_l_hand_);
      // left shoulder to left elbow
      tf_listener_->lookupTransform(hands_adaption_params_.input_l_shoulder_name,
                                   hands_adaption_params_.input_l_elbow_name,
                                   ros::Time(0), tf_input_l_shoulder_elbow_);
      // left elbow to left hand
      tf_listener_->lookupTransform(hands_adaption_params_.input_l_elbow_name,
                                   hands_adaption_params_.input_l_hand_name,
                                   ros::Time(0), tf_input_l_elbow_hand_);
      // right shoulder to left shoulder
      tf_listener_->lookupTransform(hands_adaption_params_.input_r_shoulder_name,
                                   hands_adaption_params_.input_l_shoulder_name,
                                   ros::Time(0), tf_input_r_shoulder_l_shoulder_);
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
      tf_listener_->lookupTransform(hands_adaption_params_.target_ref_name,
                                    hands_adaption_params_.target_r_shoulder_name,
                                    ros::Time(0),
                                    tf_target_torso_r_shoulder_);
      tf_listener_->lookupTransform(hands_adaption_params_.target_ref_name,
                                    hands_adaption_params_.target_l_shoulder_name,
                                    ros::Time(0),
                                    tf_target_torso_l_shoulder_);
      tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                    hands_adaption_params_.target_l_shoulder_name,
                                    ros::Time(0),
                                    tf_target_r_shoulder_l_shoulder_);
      if ((hands_adaption_params_.target_r_elbow_name != "")
          && (hands_adaption_params_.target_l_elbow_name != ""))
      {
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_shoulder_name,
                                      hands_adaption_params_.target_r_elbow_name,
                                      ros::Time(0),
                                      tf_target_r_shoulder_r_elbow_);
        tf_listener_->lookupTransform(hands_adaption_params_.target_r_elbow_name,
                                      hands_adaption_params_.target_r_hand_name,
                                      ros::Time(0),
                                      tf_target_r_elbow_r_hand_);
        use_elbows_ = true;
        ROS_DEBUG_STREAM("Motion Adaption: Will use elbows.");
      }
      else
      {
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
  bool adaptHandPositions()
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


    vec_helper_ = (tf_target_torso_r_shoulder_.getOrigin() + tf_target_torso_l_shoulder_.getOrigin()) / 2.0;
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
      ROS_DEBUG_STREAM("right hand - input position: " << tf_input_r_hand_.getOrigin().x()
                       << ", " << tf_input_r_hand_.getOrigin().y() << ", " << tf_input_r_hand_.getOrigin().z());
//      x_norm_ = tf_input_r_hand_.getOrigin().x() / (input_arm_length_ + 0.5 * input_shoulder_width_);
//      y_norm_ = tf_input_r_hand_.getOrigin().y() / (input_arm_length_ + input_shoulder_height_);
//      z_norm_ = tf_input_r_hand_.getOrigin().z() / input_arm_length_;
      x_norm_ = tf_input_r_hand_.getOrigin().x() / input_arm_length_;
      y_norm_ = (tf_input_r_hand_.getOrigin().y() - 0.5 * input_shoulder_width_) / input_arm_length_;
      z_norm_ = (tf_input_r_hand_.getOrigin().z() - input_shoulder_height_)/ input_arm_length_;
      ROS_DEBUG_STREAM("right hand - normalised position - right hand: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
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
      y_norm_ = (tf_input_l_hand_.getOrigin().y() - 0.5 * input_shoulder_width_) / input_arm_length_;
      z_norm_ = (tf_input_l_hand_.getOrigin().z() - input_shoulder_height_)/ input_arm_length_;
      ROS_DEBUG_STREAM("left hand - normalised position - right hand: " << x_norm_ << ", " << y_norm_ << ", " << z_norm_);
//      x_adapt_ = x_norm_ * ( target_arm_length_ + 0.5 * target_shoulder_width_);
//      y_adapt_ = y_norm_ * ( target_arm_length_ + target_shoulder_height_);
//      z_adapt_ = z_norm_ * target_arm_length_;
      x_adapt_ = x_norm_ * target_arm_length_;
      y_adapt_ = (y_norm_ * target_arm_length_) +  0.5 * target_shoulder_width_;
      z_adapt_ = (z_norm_ * target_arm_length_) + target_shoulder_height_;
      ROS_DEBUG_STREAM("left hand - adapted position: " << x_adapt_ << ", " << y_adapt_ << ", " << z_adapt_);
      tf_l_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

      tf_broadcaster_->sendTransform(tf::StampedTransform(tf_r_hand_scaled_, tf_input_stamp_,
                                                          hands_adaption_params_.target_ref_name, "/hand_r_pos_adapted"));
      internal_tf_->setTransform(tf::StampedTransform(tf_r_hand_scaled_, tf_input_stamp_,
                                                      hands_adaption_params_.target_ref_name, "/hand_r_pos_adapted"));
      tf_broadcaster_->sendTransform(tf::StampedTransform(tf_l_hand_scaled_, tf_input_stamp_,
                                                         hands_adaption_params_.target_ref_name, "/hand_l_pos_adapted"));
      internal_tf_->setTransform(tf::StampedTransform(tf_l_hand_scaled_, tf_input_stamp_,
                                                      hands_adaption_params_.target_ref_name, "/hand_l_pos_adapted"));
      tf_r_hand_goal_ = tf_r_hand_scaled_;
      tf_l_hand_goal_ = tf_l_hand_scaled_;
    }
    else
    {
      ROS_WARN_STREAM("Hands Adaption: One or more proportions of the input and/or target system are not valid. "
                      << "Aborting adaption!");
      return false;
    }
    return true;
  }

  /**
   *
   * @return
   */
  bool adaptShoulderOrientations()
  {
    tf_r_shoulder_scaled_.setOrigin(tf_target_torso_r_shoulder_.getOrigin());
    tf_l_shoulder_scaled_.setOrigin(tf_target_torso_l_shoulder_.getOrigin());
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/r_target_shoulder"));
    internal_tf_->setTransform(tf::StampedTransform(tf_r_shoulder_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/r_target_shoulder"));
  // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/l_target_shoulder"));
    internal_tf_->setTransform(tf::StampedTransform(tf_l_shoulder_scaled_, tf_input_stamp_,
                                                    hands_adaption_params_.target_ref_name, "/l_target_shoulder"));
    /*
    tf_r_shoulder_scaled_.setOrigin(tf::Vector3(target_shoulder_width_*0;.5, target_shoulder_height_,
    target_shoulder_x_offset_));
    tf_l_shoulder_scaled_.setOrigin(tf::Vector3(-target_shoulder_width_*0.5, target_shoulder_height_,
    target_shoulder_x_offset_));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),
    "/ref_frame", "/r_shoulder_scaled"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),
    "/ref_frame", "/l_shoulder_scaled"));
    */

    try
    {
      //tf_listener_->waitForTransform("/r_target_shoulder", "/r_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/r_target_shoulder", "/r_elbow_scaled", ros::Time(0), tf_r_shoulder_elbow_);
      //tf_listener_->waitForTransform("/r_target_shoulder", "/r_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/r_target_shoulder", "/r_hand_scaled", ros::Time(0), tf_r_shoulder_hand_);
      //tf_listener_->waitForTransform("/l_target_shoulder", "/l_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/l_target_shoulder", "/l_elbow_scaled", ros::Time(0), tf_l_shoulder_elbow_);
      //tf_listener_->waitForTransform("/l_target_shoulder", "/l_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/l_target_shoulder", "/l_hand_scaled", ros::Time(0), tf_l_shoulder_hand_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_DEBUG("%s",ex.what());
      ROS_WARN("(Step 3.4.2) Couldn't get one or more transformations from the shoulders to the hands and elbows.");
      ROS_WARN("No further processing will be done!");
      return false;
    }

    // right shoulder
    vec_shoulder_elbow_ = tf_r_shoulder_elbow_.getOrigin();
    vec_shoulder_hand_ = tf_r_shoulder_hand_.getOrigin();
    if(vec_shoulder_hand_.angle(vec_shoulder_elbow_) < 0.15708 && !(vec_r_elbow_hand_.isZero()))
    {
      ROS_DEBUG_THROTTLE(0.1, "valid right hand vector is used (angle between vectors: %f).",
      vec_shoulder_hand_.angle(vec_shoulder_elbow_));
      vec_shoulder_hand_ = vec_shoulder_elbow_ + vec_r_elbow_hand_;
      r_elbow_extended_ = true;
    }
    else
    {
      vec_r_elbow_hand_ = vec_shoulder_hand_ - vec_shoulder_elbow_;
      r_elbow_extended_ = false;
    }
    vec_normal_ = vec_shoulder_hand_.cross(vec_shoulder_elbow_);
    vec_helper_ = vec_normal_.cross(vec_shoulder_hand_);
    vec_shoulder_hand_.normalize();
    vec_helper_.normalize();
    vec_normal_.normalize();
    mat_orientation_.setValue(vec_shoulder_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                              vec_shoulder_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                              vec_shoulder_hand_.z(), vec_helper_.z(), vec_normal_.z());
    tf_r_shoulder_goal_.setBasis(mat_orientation_);
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_goal_, ros::Time::now(), "/r_target_shoulder", "/r_shoulder_adapted"));
    internal_tf_->setTransform(tf::StampedTransform(tf_r_shoulder_goal_, tf_input_stamp_, "/r_target_shoulder", "/r_shoulder_adapted"));

    // left shoulder
    vec_shoulder_elbow_ = tf_l_shoulder_elbow_.getOrigin();
    vec_shoulder_hand_ = tf_l_shoulder_hand_.getOrigin();
    if(vec_shoulder_hand_.angle(vec_shoulder_elbow_) < 0.15708 && !(vec_l_elbow_hand_.isZero()))
    {
      ROS_DEBUG_THROTTLE(0.1, "valid right hand vector is used (angle between vectors: %f).",
      vec_shoulder_hand_.angle(vec_shoulder_elbow_));
      vec_shoulder_hand_ = vec_shoulder_elbow_ + vec_l_elbow_hand_;
      l_elbow_extended_ = true;
    }
    else
    {
      vec_l_elbow_hand_ = vec_shoulder_hand_ - vec_shoulder_elbow_;
      l_elbow_extended_ = false;
    }
    vec_normal_ = vec_shoulder_hand_.cross(vec_shoulder_elbow_);
    vec_helper_ = vec_normal_.cross(vec_shoulder_hand_);
    vec_shoulder_hand_.normalize();
    vec_helper_.normalize();
    vec_normal_.normalize();
    mat_orientation_.setValue(vec_shoulder_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                              vec_shoulder_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                              vec_shoulder_hand_.z(), vec_helper_.z(), vec_normal_.z());
    tf_l_shoulder_goal_.setBasis(mat_orientation_);
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_goal_, ros::Time::now(), "/l_target_shoulder", "/l_shoulder_adapted"));
    internal_tf_->setTransform(tf::StampedTransform(tf_l_shoulder_goal_, tf_input_stamp_, "/l_target_shoulder", "/l_shoulder_adapted"));

    return true;
  }

  /**
   *
   * @return
   */
  bool adaptHandOrientations()
  {
    try
    {
     // tf_listener_->waitForTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), tf_r_elbow_orient_);
     // tf_listener_->waitForTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_->lookupTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), tf_l_elbow_orient_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_DEBUG("%s",ex.what());
      ROS_WARN("(Step 3.6) Couldn't get one or more transformations from the ref frame to the elbows.");
      ROS_WARN("No further processing will be done!");
      return false;
    }
    tf_r_hand_adjusted_.setIdentity();
    tf_r_hand_adjusted_.setOrigin(tf::Vector3(target_lower_arm_length_, 0.0, 0.0));
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_adjusted_, ros::Time::now(), "/r_elbow_adapted", "/r_hand_adapted"));
    internal_tf_->setTransform(tf::StampedTransform(tf_r_hand_adjusted_, tf_input_stamp_, "/r_elbow_adapted", "/r_hand_adapted"));
    tf_l_hand_goal_.setIdentity();
    tf_l_hand_goal_.setOrigin(tf::Vector3(target_lower_arm_length_, 0.0, 0.0));
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(), "/l_elbow_adapted", "/l_hand_adapted"));
    internal_tf_->setTransform(tf::StampedTransform(tf_l_hand_goal_, tf_input_stamp_, "/l_elbow_adapted", "/l_hand_adapted"));

    return true;
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
  }
};

} // namespace motion_adaption

#endif /* HANDS_ADAPTION_H_ */
