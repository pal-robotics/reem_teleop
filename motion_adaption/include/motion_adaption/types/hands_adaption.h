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
                  AdaptionType(hands_adaption_params.general_params,
                               tf_listener,
                               tf_broadcaster,
                               internal_tf),
                  hands_adaption_params_(hands_adaption_params)
  {
    tf_input_time_stamp_ =  ros::Time::now();
  };

  ~HandsAdaption(){};

  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    /*
     * Get all input transforms
     */
    getTransforms();
    /*
     * Then start with scaling hands and elbows
     */
//    if(scaleUserHandsAndElbows())
//    {
//      /*
//       * Next, use the scaled elbow and hand poses to adapt the output shoulder poses
//       */
//      if(adaptShoulders())
//      {
//        /*
//         * and to adapt the output elbow poses
//         */
//        if(adaptElbows())
//        {
//          /*
//           * and to adapt the output hand poses
//           */
//          if(!adaptHands())
//          {
//            return false;
//          }
//        }
//        else
//        {
//          return false;
//        }
//      }
//      else
//      {
//        return false;
//      }
//    }
//    else
//    {
//      return false;
//    }
    convertTfToPoseMsg(adapted_transforms);
    return true;
  };

private:
  HandsAdaptionParameters hands_adaption_params_;
  tf::TransformStamped tf_usr_torso_;
  tf::TransformStamped tf_usr_r_shoulder_,tf_usr_r_shoulder_elbow_, tf_r_shoulder_elbow_, tf_r_shoulder_hand_, tf_usr_r_elbow_, tf_usr_r_elbow_hand_, tf_usr_r_hand_;
  tf::TransformStamped tf_usr_l_shoulder_,tf_usr_l_shoulder_elbow_, tf_l_shoulder_elbow_, tf_l_shoulder_hand_, tf_usr_l_elbow_, tf_usr_l_elbow_hand_, tf_usr_l_hand_;
  tf::TransformStamped tf_robot_r_shoulder_l_shoulder_;
  tf::TransformStamped tf_robot_torso_r_shoulder_, tf_robot_r_shoulder_r_elbow_, tf_robot_r_elbow_r_hand_;
  tf::TransformStamped tf_robot_torso_l_shoulder_, tf_robot_l_shoulder_l_elbow_, tf_robot_l_elbow_l_hand_;
  tf::TransformStamped tf_r_shoulder_scaled_, tf_r_elbow_scaled_, tf_r_hand_scaled_;
  tf::TransformStamped tf_l_shoulder_scaled_, tf_l_elbow_scaled_, tf_l_hand_scaled_;
  tf::TransformStamped tf_r_elbow_pos_, tf_r_elbow_orient_, tf_r_elbow_hand_;
  tf::TransformStamped tf_l_elbow_pos_, tf_l_elbow_orient_, tf_l_elbow_hand_;
  tf::TransformStamped tf_r_hand_adjusted_, tf_l_hand_adjusted_;
  tf::TransformStamped tf_r_shoulder_goal_, tf_r_elbow_goal_, tf_r_hand_goal_;
  tf::TransformStamped tf_l_shoulder_goal_, tf_l_elbow_goal_, tf_l_hand_goal_;
  tf::Vector3 vec_shoulder_elbow_, vec_shoulder_hand_, vec_elbow_hand_,vec_normal_, vec_helper_;
  tf::Vector3 vec_r_elbow_hand_, vec_l_elbow_hand_;
  tf::Matrix3x3 mat_orientation_;
  double user_shoulder_width_, user_shoulder_height_, user_upper_arm_length_, user_arm_length_;
  double robot_shoulder_width_, robot_shoulder_heigth_, robot_upper_arm_length_, robot_lower_arm_length_, robot_arm_length_;
  double x_norm_, y_norm_, z_norm_, x_adapt_, y_adapt_, z_adapt_, elbow_x_, elbow_y_, limb_length_;
  bool r_elbow_extended_, l_elbow_extended_;
  ros::Time tf_input_stamp_;

  /**
   *
   * @return
   */
  bool getTransforms()
  {
    try
    {
      // target reference to torso
      tf_listener_.lookupTransform(hands_adaption_params_.general_params.target_ref_frame,
                                   hands_adaption_params_.user_torso_name,
                                   ros::Time(0), tf_usr_torso_);
      tf_input_stamp_ = std::min(tf_input_stamp_, tf_usr_torso_.stamp_);
      // torso to right shoulder
      tf_listener_.lookupTransform(hands_adaption_params_.user_torso_name,
                                   hands_adaption_params_.user_r_shoulder_name,
                                   ros::Time(0), tf_usr_r_shoulder_);
      // torso to right elbow
      tf_listener_.lookupTransform(hands_adaption_params_.user_torso_name,
                                   hands_adaption_params_.user_r_elbow_name,
                                   ros::Time(0), tf_usr_r_elbow_);
      // torso to right hand
      tf_listener_.lookupTransform(hands_adaption_params_.user_torso_name,
                                   hands_adaption_params_.user_r_hand_name,
                                   ros::Time(0), tf_usr_r_hand_);
      // right shoulder to right elbow
      tf_listener_.lookupTransform(hands_adaption_params_.user_r_shoulder_name,
                                   hands_adaption_params_.user_r_elbow_name,
                                   ros::Time(0), tf_usr_r_shoulder_elbow_);
      // right elbow to right hand
      tf_listener_.lookupTransform(hands_adaption_params_.user_r_elbow_name,
                                   hands_adaption_params_.user_r_hand_name,
                                   ros::Time(0), tf_usr_r_elbow_hand_);
      // torso to left shoulder
      tf_listener_.lookupTransform(hands_adaption_params_.user_l_shoulder_name,
                                   hands_adaption_params_.user_torso_name,
                                   ros::Time(0), tf_usr_l_shoulder_);
      // torso to left elbow
      tf_listener_.lookupTransform(hands_adaption_params_.user_torso_name,
                                   hands_adaption_params_.user_l_elbow_name,
                                   ros::Time(0), tf_usr_l_elbow_);
      // torso to left hand
      tf_listener_.lookupTransform(hands_adaption_params_.user_torso_name,
                                   hands_adaption_params_.user_l_hand_name,
                                   ros::Time(0), tf_usr_l_hand_);
      // left shoulder to left elbow
      tf_listener_.lookupTransform(hands_adaption_params_.user_l_shoulder_name,
                                   hands_adaption_params_.user_l_elbow_name,
                                   ros::Time(0), tf_usr_l_shoulder_elbow_);
      // left elbow to left hand
      tf_listener_.lookupTransform(hands_adaption_params_.user_l_elbow_name,
                                   hands_adaption_params_.user_l_hand_name,
                                   ros::Time(0), tf_usr_l_elbow_hand_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("hands adaption: Couldn't get one or more user transformations. Aborting adaption!");
      ROS_DEBUG_STREAM("hands adaption: tf error: " << ex.what());
      return false;
    }
    return true;
  }

  /**
   *
   * @return
   */
  bool scaleElbowsAndShoulders()
  {
    user_shoulder_width_ = tf_usr_r_shoulder_.getOrigin().x() + (- tf_usr_l_shoulder_.getOrigin().x());
    user_shoulder_height_ = (tf_usr_r_shoulder_.getOrigin().y() + tf_usr_l_shoulder_.getOrigin().y()) / 2.0;
    user_upper_arm_length_ = tf_usr_r_shoulder_elbow_.getOrigin().x();
    user_arm_length_ = tf_usr_r_shoulder_elbow_.getOrigin().x() + tf_usr_r_elbow_hand_.getOrigin().x();

    if (user_shoulder_width_ > 0.0 && user_shoulder_height_ > 0.0 && user_arm_length_ > 0.0)
    {
      x_norm_ = tf_usr_r_hand_.getOrigin().x() / (user_arm_length_ + 0.5 * user_shoulder_width_);
      y_norm_ = tf_usr_r_hand_.getOrigin().y() / (user_arm_length_ + user_shoulder_height_);
      z_norm_ = tf_usr_r_hand_.getOrigin().z() / user_arm_length_;
      x_adapt_ = x_norm_ * ( robot_arm_length_ + 0.5 * robot_shoulder_width_);
      y_adapt_ = y_norm_ * ( robot_arm_length_ + robot_shoulder_heigth_);
      z_adapt_ = z_norm_ * robot_arm_length_;
      tf_r_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

      x_norm_ = tf_usr_l_hand_.getOrigin().x() / (user_arm_length_ + 0.5 * user_shoulder_width_);
      y_norm_ = tf_usr_l_hand_.getOrigin().y() / (user_arm_length_ + user_shoulder_height_);
      z_norm_ = tf_usr_l_hand_.getOrigin().z() / user_arm_length_;
      x_adapt_ = x_norm_ * ( robot_arm_length_ + 0.5 * robot_shoulder_width_);
      y_adapt_ = y_norm_ * ( robot_arm_length_ + robot_shoulder_heigth_);
      z_adapt_ = z_norm_ * robot_arm_length_;
      tf_l_hand_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

      x_norm_ = tf_usr_r_elbow_.getOrigin().x() / (user_upper_arm_length_ + 0.5 * user_shoulder_width_);
      y_norm_ = tf_usr_r_elbow_.getOrigin().y() / (user_upper_arm_length_ + user_shoulder_height_);
      z_norm_ = tf_usr_r_elbow_.getOrigin().z() / user_upper_arm_length_;
      x_adapt_ = x_norm_ * ( robot_upper_arm_length_ + 0.5 * robot_shoulder_width_);
      y_adapt_ = y_norm_ * ( robot_upper_arm_length_ + robot_shoulder_heigth_);
      z_adapt_ = z_norm_ * robot_upper_arm_length_;
      tf_r_elbow_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

      x_norm_ = tf_usr_l_elbow_.getOrigin().x() / (user_upper_arm_length_ + 0.5 * user_shoulder_width_);
      y_norm_ = tf_usr_l_elbow_.getOrigin().y() / (user_upper_arm_length_ + user_shoulder_height_);
      z_norm_ = tf_usr_l_elbow_.getOrigin().z() / user_upper_arm_length_;
      x_adapt_ = x_norm_ * ( robot_upper_arm_length_ + 0.5 * robot_shoulder_width_);
      y_adapt_ = y_norm_ * ( robot_upper_arm_length_ + robot_shoulder_heigth_);
      z_adapt_ = z_norm_ * robot_upper_arm_length_;
      tf_l_elbow_scaled_.setOrigin(tf::Vector3(x_adapt_, y_adapt_, z_adapt_));

   //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_scaled_, ros::Time::now(), "/ref_frame", "/r_elbow_scaled"));
      internal_tf_.setTransform(tf::StampedTransform(tf_r_elbow_scaled_, tf_input_stamp_,
                                                     adaption_parameters_.input_ref_frame, "/r_elbow_scaled"));
   //   tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_scaled_, ros::Time::now(), "/ref_frame", "/r_hand_scaled"));
      internal_tf_.setTransform(tf::StampedTransform(tf_r_hand_scaled_, tf_input_stamp_,
                                                     adaption_parameters_.input_ref_frame, "/r_hand_scaled"));
    //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_scaled_, ros::Time::now(), "/ref_frame", "/l_elbow_scaled"));
      internal_tf_.setTransform(tf::StampedTransform(tf_l_elbow_scaled_, tf_input_stamp_,
                                                     adaption_parameters_.input_ref_frame, "/l_elbow_scaled"));
     // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_scaled_, ros::Time::now(), "/ref_frame", "/l_hand_scaled"));
      internal_tf_.setTransform(tf::StampedTransform(tf_l_hand_scaled_, tf_input_stamp_,
                                                     adaption_parameters_.input_ref_frame, "/l_hand_scaled"));
    }
    else
    {
      ROS_WARN_STREAM("hands adaption: User's body proportions are not valid. Aborting adaption!");
      return false;
    }
    return true;
  }

  /**
   *
   * @return
   */
  bool adaptShoulders()
  {
    // positions
    try
    {
      tf_listener_.waitForTransform(adaption_parameters_.input_ref_frame, robot_r_shoulder_str_, ros::Time(0),
       ros::Duration(wait_for_tf_));
      tf_listener_.lookupTransform(adaption_parameters_.input_ref_frame, robot_r_shoulder_str_, ros::Time(0),
      tf_robot_torso_r_shoulder_);
      tf_listener_.waitForTransform(adaption_parameters_.input_ref_frame, robot_l_shoulder_str_, ros::Time(0),
      ros::Duration(wait_for_tf_));
      tf_listener_.lookupTransform(adaption_parameters_.input_ref_frame, robot_l_shoulder_str_, ros::Time(0),
      tf_robot_torso_l_shoulder_);
      tf_listener_.waitForTransform(robot_r_shoulder_str_, robot_l_shoulder_str_, ros::Time(0),
      ros::Duration(wait_for_tf_));
      tf_listener_.lookupTransform(robot_r_shoulder_str_, robot_l_shoulder_str_, ros::Time(0),
      tf_robot_r_shoulder_l_shoulder_);
      tf_listener_.waitForTransform(robot_r_shoulder_str_, robot_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
      tf_listener_.lookupTransform(robot_r_shoulder_str_, robot_r_elbow_str_, ros::Time(0), tf_robot_r_shoulder_r_elbow_);
      tf_listener_.waitForTransform(robot_r_elbow_str_, robot_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
      tf_listener_.lookupTransform(robot_r_elbow_str_, robot_r_hand_str_, ros::Time(0), tf_robot_r_elbow_r_hand_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_DEBUG("%s",ex.what());
      ROS_WARN("(Step 3.4.1) Couldn't get one or more transformations of the robot to calculate body measurements.");
      ROS_WARN("No further processing will be done!");
      return false;
    }
    robot_shoulder_heigth_ = sqrt(pow(tf_robot_torso_r_shoulder_.getOrigin()[
    tf_robot_torso_r_shoulder_.getOrigin().absolute().maxAxis()], 2));
    robot_shoulder_width_ = sqrt(pow(tf_robot_r_shoulder_l_shoulder_.getOrigin()[
                                 tf_robot_r_shoulder_l_shoulder_.getOrigin().absolute().maxAxis()], 2));
    robot_upper_arm_length_ = sqrt(tf_robot_r_shoulder_r_elbow_.getOrigin().length2());
    robot_lower_arm_length_ = sqrt(tf_robot_r_elbow_r_hand_.getOrigin().length2());
    robot_arm_length_ = robot_upper_arm_length_ + robot_lower_arm_length_;
    ROS_DEBUG_STREAM_THROTTLE(1.0, "robot_shoulder_heigth = " << robot_shoulder_heigth_
                                   << ", robot_shoulder_width = " << robot_shoulder_width_
                                   << ", robot_arm_length = " << robot_arm_length_);
    tf_r_shoulder_scaled_.setOrigin(tf_robot_torso_r_shoulder_.getOrigin());
    tf_l_shoulder_scaled_.setOrigin(tf_robot_torso_l_shoulder_.getOrigin());
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/r_robot_shoulder"));
    internal_tf_.setTransform(tf::StampedTransform(tf_r_shoulder_scaled_, tf_input_stamp_,"/ref_frame", "/r_robot_shoulder"));
  // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),"/ref_frame", "/l_robot_shoulder"));
    internal_tf_.setTransform(tf::StampedTransform(tf_l_shoulder_scaled_, tf_input_stamp_,"/ref_frame", "/l_robot_shoulder"));
    /*
    tf_r_shoulder_scaled_.setOrigin(tf::Vector3(robot_shoulder_width_*0;.5, robot_shoulder_heigth_,
    robot_shoulder_x_offset_));
    tf_l_shoulder_scaled_.setOrigin(tf::Vector3(-robot_shoulder_width_*0.5, robot_shoulder_heigth_,
    robot_shoulder_x_offset_));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_scaled_, ros::Time::now(),
    "/ref_frame", "/r_shoulder_scaled"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_scaled_, ros::Time::now(),
    "/ref_frame", "/l_shoulder_scaled"));
    */

    // orientations
    try
    {
      //tf_listener_.waitForTransform("/r_robot_shoulder", "/r_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/r_robot_shoulder", "/r_elbow_scaled", ros::Time(0), tf_r_shoulder_elbow_);
      //tf_listener_.waitForTransform("/r_robot_shoulder", "/r_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/r_robot_shoulder", "/r_hand_scaled", ros::Time(0), tf_r_shoulder_hand_);
      //tf_listener_.waitForTransform("/l_robot_shoulder", "/l_elbow_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/l_robot_shoulder", "/l_elbow_scaled", ros::Time(0), tf_l_shoulder_elbow_);
      //tf_listener_.waitForTransform("/l_robot_shoulder", "/l_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/l_robot_shoulder", "/l_hand_scaled", ros::Time(0), tf_l_shoulder_hand_);
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
    if(vec_shoulder_hand_.angle(vec_shoulder_elbow_) < 0.15708 && !(vec_r_elbow_hand_valid_.isZero()))
    {
      ROS_DEBUG_THROTTLE(0.1, "valid right hand vector is used (angle between vectors: %f).",
      vec_shoulder_hand_.angle(vec_shoulder_elbow_));
      vec_shoulder_hand_ = vec_shoulder_elbow_ + vec_r_elbow_hand_valid_;
      r_elbow_extended_ = true;
    }
    else
    {
      vec_r_elbow_hand_valid_ = vec_shoulder_hand_ - vec_shoulder_elbow_;
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
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_shoulder_goal_, ros::Time::now(), "/r_robot_shoulder", "/r_shoulder_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_r_shoulder_goal_, tf_input_stamp_, "/r_robot_shoulder", "/r_shoulder_adapted"));

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
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_shoulder_goal_, ros::Time::now(), "/l_robot_shoulder", "/l_shoulder_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_l_shoulder_goal_, tf_input_stamp_, "/l_robot_shoulder", "/l_shoulder_adapted"));

    return true;
  }

  /**
   *
   * @return
   */
  bool adaptElbows()
  {
    // positions
    limb_length_ = tf_r_shoulder_hand_.getOrigin().length();
    if (limb_length_ >= robot_arm_length_|| r_elbow_extended_)
    {
      elbow_x_ = robot_upper_arm_length_;
      elbow_y_ = 0.0;
    }
    else if (limb_length_ < 1e-6)
    {
      elbow_x_ = 0.0;
      elbow_y_ = robot_upper_arm_length_;
    }
    else
    {
      elbow_x_ = (pow(limb_length_, 2) - pow(robot_lower_arm_length_, 2) + pow(robot_upper_arm_length_, 2)) /
      ( 2 * limb_length_);
      elbow_y_ = sqrt(pow(robot_upper_arm_length_, 2) - pow(elbow_x_, 2));
    }
    tf_r_elbow_pos_.setOrigin(tf::Vector3(elbow_x_, elbow_y_, 0.0));
    //tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_pos_, ros::Time::now(), "/r_shoulder_adapted", "/r_elbow_pos"));
    internal_tf_.setTransform(tf::StampedTransform(tf_r_elbow_pos_, tf_input_stamp_, "/r_shoulder_adapted", "/r_elbow_pos"));

    limb_length_ = tf_l_shoulder_hand_.getOrigin().length();
    if (limb_length_ >= robot_arm_length_ || l_elbow_extended_)
    {
      elbow_x_ = robot_upper_arm_length_;
      elbow_y_ = 0.0;
    }
    else if (limb_length_ < 1e-6)
    {
      elbow_x_ = 0.0;
      elbow_y_ = robot_upper_arm_length_;
    }
    else
    {
      elbow_x_ = (pow(limb_length_, 2) - pow(robot_lower_arm_length_, 2) + pow(robot_upper_arm_length_, 2)) /
      ( 2 * limb_length_);
      elbow_y_ = sqrt(pow(robot_upper_arm_length_, 2) - pow(elbow_x_, 2));
    }
    tf_l_elbow_pos_.setOrigin(tf::Vector3(elbow_x_, elbow_y_, 0.0));
  //  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_pos_, ros::Time::now(), "/l_shoulder_adapted", "/l_elbow_pos"));
    internal_tf_.setTransform(tf::StampedTransform(tf_l_elbow_pos_, tf_input_stamp_, "/l_shoulder_adapted", "/l_elbow_pos"));

    // orientations
    try
    {
     // tf_listener_.waitForTransform("/r_elbow_pos", "/r_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/r_elbow_pos", "/r_hand_scaled", ros::Time(0), tf_r_elbow_hand_);
     // tf_listener_.waitForTransform("/l_elbow_pos", "/l_hand_scaled", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/l_elbow_pos", "/l_hand_scaled", ros::Time(0), tf_l_elbow_hand_);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("%s",ex.what());
      ROS_WARN("(Step 3.5) Couldn't get one or more transformations from the elbow to the hands.");
      ROS_WARN("No further processing will be done!");
      return false;
    }

    // this is already done further up
//    if(l_elbow_extended_ == true)
//    {
//      tf_l_elbow_hand_.setOrigin(tf::Vector3(robot_lower_arm_length_, 0.0, 0.0));
//    }
    vec_elbow_hand_ = tf_r_elbow_hand_.getOrigin();
    vec_normal_ = tf::Vector3(0.0, 0.0, 1.0);
    vec_helper_ = vec_normal_.cross(vec_elbow_hand_);
    vec_elbow_hand_.normalize();
    vec_normal_.normalize();
    vec_helper_.normalize();
    mat_orientation_.setValue(vec_elbow_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                              vec_elbow_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                              vec_elbow_hand_.z(), vec_helper_.z(), vec_normal_.z());
    tf_r_elbow_goal_.setOrigin(tf_r_elbow_pos_.getOrigin());
    tf_r_elbow_goal_.setBasis(mat_orientation_);

   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_elbow_goal_, ros::Time::now(), "/r_shoulder_adapted", "/r_elbow_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_r_elbow_goal_, tf_input_stamp_, "/r_shoulder_adapted", "/r_elbow_adapted"));
    vec_elbow_hand_ = tf_l_elbow_hand_.getOrigin();
    vec_normal_ = tf::Vector3(0.0, 0.0, 1.0);
    vec_helper_ = vec_normal_.cross(vec_elbow_hand_);
    vec_elbow_hand_.normalize();
    vec_normal_.normalize();
    vec_helper_.normalize();
    mat_orientation_.setValue(vec_elbow_hand_.x(), vec_helper_.x(), vec_normal_.x(),
                              vec_elbow_hand_.y(), vec_helper_.y(), vec_normal_.y(),
                              vec_elbow_hand_.z(), vec_helper_.z(), vec_normal_.z());
    tf_l_elbow_goal_.setOrigin(tf_l_elbow_pos_.getOrigin());
    tf_l_elbow_goal_.setBasis(mat_orientation_);
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_elbow_goal_, ros::Time::now(), "/l_shoulder_adapted", "/l_elbow_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_l_elbow_goal_, tf_input_stamp_, "/l_shoulder_adapted", "/l_elbow_adapted"));

    return true;
  }

  /**
   *
   * @return
   */
  bool adaptHands()
  {
    try
    {
     // tf_listener_.waitForTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/ref_frame", "/r_elbow_adapted", ros::Time(0), tf_r_elbow_orient_);
     // tf_listener_.waitForTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), ros::Duration(wait_for_tf_));
      internal_tf_.lookupTransform("/ref_frame", "/l_elbow_adapted", ros::Time(0), tf_l_elbow_orient_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_DEBUG("%s",ex.what());
      ROS_WARN("(Step 3.6) Couldn't get one or more transformations from the ref frame to the elbows.");
      ROS_WARN("No further processing will be done!");
      return false;
    }
    tf_r_hand_adjusted_.setIdentity();
    tf_r_hand_adjusted_.setOrigin(tf::Vector3(robot_lower_arm_length_, 0.0, 0.0));
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_r_hand_adjusted_, ros::Time::now(), "/r_elbow_adapted", "/r_hand_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_r_hand_adjusted_, tf_input_stamp_, "/r_elbow_adapted", "/r_hand_adapted"));
    tf_l_hand_goal_.setIdentity();
    tf_l_hand_goal_.setOrigin(tf::Vector3(robot_lower_arm_length_, 0.0, 0.0));
   // tf_broadcaster_.sendTransform(tf::StampedTransform(tf_l_hand_goal_, ros::Time::now(), "/l_elbow_adapted", "/l_hand_adapted"));
    internal_tf_.setTransform(tf::StampedTransform(tf_l_hand_goal_, tf_input_stamp_, "/l_elbow_adapted", "/l_hand_adapted"));

    return true;
  }

  /**
   *
   */
  void convertTfToPoseMsg(std::vector<geometry_msgs::PoseStamped>& adapted_transforms)
  {
    // Torso
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.goal_torso_name;
    pose_adapted_.pose.position.x = tf_torso_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_torso_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_torso_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_torso_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_torso_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_torso_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_torso_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);

    // Right shoulder
//    pose_adapted_.header.stamp = ros::Time::now();
//    pose_adapted_.header.frame_id = hands_adaption_params_.goal_r_shoulder_name;
//    pose_adapted_.pose.position.x = tf_r_hand_goal_.getOrigin().x();
//    pose_adapted_.pose.position.y = tf_r_hand_goal_.getOrigin().y();
//    pose_adapted_.pose.position.z = tf_r_hand_goal_.getOrigin().z();
//    pose_adapted_.pose.orientation.x = tf_r_hand_goal_.getRotation().x();
//    pose_adapted_.pose.orientation.y = tf_r_hand_goal_.getRotation().y();
//    pose_adapted_.pose.orientation.z = tf_r_hand_goal_.getRotation().z();
//    pose_adapted_.pose.orientation.w = tf_r_hand_goal_.getRotation().w();
//    adapted_transforms.push_back(pose_adapted_);

    // Right elbow
    pose_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.goal_r_elbow_name;
    pose_adapted_.pose.position.x = tf_r_elbow_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_r_elbow_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_r_elbow_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_r_elbow_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_r_elbow_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_r_elbow_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_r_elbow_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);

    // Right hand
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.goal_r_hand_name;
    pose_adapted_.pose.position.x = tf_r_hand_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_r_hand_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_r_hand_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_r_hand_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_r_hand_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_r_hand_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_r_hand_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);

    // Left shoulder
//    pose_adapted_.header.stamp = ros::Time::now();
//    pose_adapted_.header.frame_id = hands_adaption_params_.goal_r_hand_name;
//    pose_adapted_.pose.position.x = tf_r_hand_goal_.getOrigin().x();
//    pose_adapted_.pose.position.y = tf_r_hand_goal_.getOrigin().y();
//    pose_adapted_.pose.position.z = tf_r_hand_goal_.getOrigin().z();
//    pose_adapted_.pose.orientation.x = tf_r_hand_goal_.getRotation().x();
//    pose_adapted_.pose.orientation.y = tf_r_hand_goal_.getRotation().y();
//    pose_adapted_.pose.orientation.z = tf_r_hand_goal_.getRotation().z();
//    pose_adapted_.pose.orientation.w = tf_r_hand_goal_.getRotation().w();
//    adapted_transforms.push_back(pose_adapted_);

    // Left elbow
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.goal_l_elbow_name;
    pose_adapted_.pose.position.x = tf_l_elbow_goal_.getOrigin().x();
    pose_adapted_.pose.position.y = tf_l_elbow_goal_.getOrigin().y();
    pose_adapted_.pose.position.z = tf_l_elbow_goal_.getOrigin().z();
    pose_adapted_.pose.orientation.x = tf_l_elbow_goal_.getRotation().x();
    pose_adapted_.pose.orientation.y = tf_l_elbow_goal_.getRotation().y();
    pose_adapted_.pose.orientation.z = tf_l_elbow_goal_.getRotation().z();
    pose_adapted_.pose.orientation.w = tf_l_elbow_goal_.getRotation().w();
    adapted_transforms.push_back(pose_adapted_);

    // Left hand
    pose_adapted_.header.stamp = ros::Time::now();
    pose_adapted_.header.frame_id = hands_adaption_params_.goal_l_hand_name;
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
