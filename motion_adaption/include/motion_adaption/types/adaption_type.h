/**
 * @file /motion_adaption/include/motion_adaption/types/adaption_type.h
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

#ifndef ADAPTION_TYPE_H_
#define ADAPTION_TYPE_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
//#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include "adaption_parameters.h"

namespace motion_adaption
{

class AdaptionType
{
public:
  /**
   * Generic adaption type definition, which contains the basic information and tools for all adaption types.
   * @param adaption_parameters
   * @param tf_listener
   */
  AdaptionType(const AdaptionParameters& adaption_params,
                 boost::shared_ptr<tf::TransformListener> tf_listener,
                 boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                 boost::shared_ptr<tf::Transformer> internal_tf) :
                 adaption_params_(adaption_params),
                 tf_listener_(tf_listener),
                 tf_broadcaster_(tf_broadcaster),
                 internal_tf_(internal_tf)
  {
    std::cout << "AdaptionType constructor: adaption name: " << adaption_params_.adaption_name << std::endl;
  };
  /**
   * Cleans up
   */
  virtual ~AdaptionType(){};

  /**
   * Every derived adaption type needs to implement this function
   */
  virtual bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_transforms) = 0;

  /**
   * Get a reference to the name of the configured adaption
   * @return name reference
   */
  const std::string& getAdaptionName() const
  {
    return adaption_params_.adaption_name;
  };

protected:
  /**
   * Set the reference frame
   * The input reference frame will be located at input_ref_dep_child's position,
   * but with the orientation of the target system. This will make the following adaption easier.
   * @return true, if no errors occur
   */
  bool setReferenceFrame()
  {
    try
    {
      tf_listener_->lookupTransform(adaption_params_.target_ref_name,
                                    adaption_params_.input_pos_ref_name,
                                    ros::Time(0),
                                    tf_input_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("Coulnd't get transform from '" << adaption_params_.target_ref_name
                      << "' to '" << adaption_params_.input_pos_ref_name
                      << "'! Skipping motion adaption.");
      ROS_DEBUG_STREAM(ex.what());
      return false;
    }
    // Use the same position
    tf_adapted_ = tf_input_;
    // But adjust the orientation to align with the target system - has to be done manually.
    // TODO: should be possible to determine automatically!
    quat_ = tf::createIdentityQuaternion ();
    quat_adapt_.setRPY(adaption_params_.input_ref_orient_adjust.roll,
                       adaption_params_.input_ref_orient_adjust.pitch,
                       adaption_params_.input_ref_orient_adjust.yaw);
    tf_adapted_.setRotation(quat_ * quat_adapt_);
    tf_broadcaster_->sendTransform(tf::StampedTransform(tf_adapted_,
                                                        ros::Time::now(),
                                                        adaption_params_.target_ref_name,
                                                        adaption_params_.input_ref_name));
    return true;
  }
//  /**
//   * Adaption parameters
//   */
  AdaptionParameters adaption_params_;
  /**
   * Transform listener for retrieving all necessary transformations
   */
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  /**
   * Transform broadcaster for publishing the results of adaption (optional)
   */
  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  /**
   * TF transformer for internal calculations
   */
  boost::shared_ptr<tf::Transformer> internal_tf_;
  /**
   * Input, target and adapted transforms
   */
  tf::StampedTransform tf_input_, tf_target_, tf_adapted_;
  /**
   * Quaternion for internal processing
   */
  tf::Quaternion quat_, quat_adapt_;
  /**
   * Vector for internal processing
   */
  tf::Vector3 vec_;
  /**
   * Used for converting transforms into messages
   */
  geometry_msgs::PoseStamped pose_adapted_;
};

typedef boost::shared_ptr<AdaptionType> AdaptionTypePtr;

} // namespace motion_adaption

#endif /* ADAPTION_TYPE_H_ */
