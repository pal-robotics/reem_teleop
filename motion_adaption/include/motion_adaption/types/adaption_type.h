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

namespace motion_adaption
{

struct MotionAdaptionParameters
{
  enum MotionAdaptionType
  {
    NoAdaption,
    TransRotAdaption
  };
  std::string adaption_name;
  MotionAdaptionType adaption_type;
  std::string input_ref_frame;
  std::vector<std::string> input_endpts;
  std::string target_ref_frame;
  std::vector<std::string> target_endpts;
  std::vector<std::string> goal_endpts;
  struct InputCorrection
  {
    double roll;
    double pitch;
    double yaw;
  };
  InputCorrection input_correction;
  double wait_for_tf;
  double tf_calc_time;
};

class AdaptionType
{
public:
  /**
   * Generic adaption type definition, which contains the basic information and tools for all adaption types.
   * @param adaption_parameters
   * @param tf_listener
   */
  AdaptionType(const MotionAdaptionParameters& adaption_parameters,
                 boost::shared_ptr<tf::TransformListener> tf_listener,
                 boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster,
                 boost::shared_ptr<tf::Transformer> internal_tf) :
                 adaption_parameters_(adaption_parameters),
                 tf_listener_(tf_listener),
                 tf_broadcaster_(tf_broadcaster),
                 internal_tf_(internal_tf),
                 wait_for_tf_(adaption_parameters.wait_for_tf),
                 tf_calc_time_(adaption_parameters.tf_calc_time)
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
    return adaption_parameters_.adaption_name;
  };

protected:
//private:
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
//private:
  /**
   * Adaption parameters
   */
  MotionAdaptionParameters adaption_parameters_;
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
   * Time the transform listeners waits for retrieving a transform
   */
  ros::Duration wait_for_tf_;
  /**
   * Time the internal tf transformer may use for calculations
   */
  ros::Duration tf_calc_time_;
  /**
   *
   */
  geometry_msgs::PoseStamped pose_adapted_;
};

typedef boost::shared_ptr<AdaptionType> AdaptionTypePtr;

} // namespace motion_adaption

#endif /* ADAPTION_TYPE_H_ */
