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
//  AdaptionType(std::string adaption_name,
//                 AdaptionTypes adaption_type,
//                 double wait_for_tf,
//                 std::string input_ref_frame,
//                 std::string input_ref_dep_parent,
//                 std::string input_ref_dep_child,
//                 OrientationCorrection input_ref_correction,
//                 std::string target_ref_frame,
//                 boost::shared_ptr<tf::TransformListener> tf_listener,
//                 boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
//                 boost::shared_ptr<tf::Transformer> internal_tf) :
//                 adaption_name_(adaption_name),
//                 adaption_type_(adaption_type),
//                 wait_for_tf_(wait_for_tf),
//                 input_ref_frame_(input_ref_frame),
//                 input_ref_dep_parent_(input_ref_dep_parent),
//                 input_ref_dep_child_(input_ref_dep_child),
//                 input_ref_correction_(input_ref_correction),
//                 target_ref_frame_(target_ref_frame),
//                 tf_listener_(tf_listener),
//                 tf_broadcaster_(tf_broadcaster),
//                 internal_tf_(internal_tf)
  AdaptionType(const GeneralParameters& general_params,
                 boost::shared_ptr<tf::TransformListener> tf_listener,
                 boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                 boost::shared_ptr<tf::Transformer> internal_tf) :
                 general_params_(general_params),
                 tf_listener_(tf_listener),
                 tf_broadcaster_(tf_broadcaster),
                 internal_tf_(internal_tf)
  {};
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
    return general_params_.adaption_name;
  };

protected:
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
   * Adaption parameters
   */
  GeneralParameters general_params_;
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
