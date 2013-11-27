/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_retargeting/LICENSE
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_RETARGETING_MOTION_RETARGETING_H_
#define MOTION_RETARGETING_MOTION_RETARGETING_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <motion_adaption/motion_adaption.h>
#include <tree_kinematics/tree_kinematics.h>
#include <tree_kinematics/GetTreePositionIK.h>

#include "collision_checking/collision_checker.h"
#include "motion_recorder/motion_recorder.h"
#include "output_handler/output_handler.h"

namespace motion_retargeting
{

class MotionRetargeting
{
public:
  /**
   * Initialises motion retargeting by configuring motion adaption, tree kinematics and the output handler,
   * as well as collision checking and motion recording, if defined.
   *
   * @param nh normal node handle for the joint states subscriber
   * @param nh_private private node handle for registering motion retargeting control subscribers
   * @param motion_adaption pointer to motion adaption
   * @param kinematics_params kinematics parameters used for the calls to the IK solver
   * @param tree_kinematics pointer to tree kinematics
   * @param collision_checker pointer to the collision checker
   * @param motion_recorder pointer to the motion recorder
   * @param output_handler pointer to the output handler
   */
  MotionRetargeting(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private,
                    const motion_adaption::MotionAdaptionPtr motion_adaption,
                    const tree_kinematics::KinematicsParameters& kinematics_params,
                    const tree_kinematics::TreeKinematicsPtr tree_kinematics,
                    const CollisionCheckerPtr collision_checker,
                    const OutputHandlerPtr output_handler,
                    const MotionRecorderPtr motion_recorder);
  /**
   * Throws out the trash
   */
  ~MotionRetargeting();
  /**
   * Does a one shot retargeting of the input task space motion to the output joint space motion
   *
   * @return true, if now error occurred
   *
   * TODO:
   *   * Current input (retrieveing tf transforms) is handled by motion adaption.
   *     Should we outsource that and make it the responsibility of the program using this class?
   *     * Pros: more flexible
   *     * Cons: needs checking of correct input and output each time -> takes time
   */
  bool retarget();

private:
  /**
   * ROS node handle for the joint states subscriber
   */
  ros::NodeHandle nh_;
  /**
   * Private ROS node handle for motion retargeting control topics (e.g. (de)activate motion recording)
   */
  ros::NodeHandle nh_private_;
  /**
   * Motion adaption for adapting the input motion to the target system (e.g. robot)
   */
  motion_adaption::MotionAdaptionPtr motion_adaption_;
  /**
   * Adapted endpoint poses
   */
  std::vector<geometry_msgs::PoseStamped> adapted_entpt_poses_;
  /**
   * Tree IK for calculating the goal joint states
   */
  tree_kinematics::TreeKinematicsPtr tree_kinematics_;
  /**
   * Tree IK service request
   */
  tree_kinematics::GetTreePositionIK::Request tree_ik_request_;
  /**
   * Tree IK service response
   */
  tree_kinematics::GetTreePositionIK::Response tree_ik_response_;
  /**
   * Output handler for publishing the goal joint states
   */
  OutputHandlerPtr output_handler_;
  /**
   * Goal joint states sent to the output handler
   */
  sensor_msgs::JointState goal_joint_states_;
  /**
   * Subscriber for retrieving the current joint states, which are used as seed state for the IK calculations
   */
  ros::Subscriber joint_states_subscriber_;
  /**
   * Storage for last joint states
   */
  sensor_msgs::JointState joint_states_;
  /**
   * Flag for joint state initialisation. It is set to true, once first joint states have been received.
   */
  bool joint_states_initialised_;
  /**
   * Callback function for the joint states subscriber
   *
   * @param msg incoming joint states
   */
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  /**
   * Pointer to the collision checker
   */
  CollisionCheckerPtr collision_checker_;
  /**
   * Subscriber for turning on and off collision checking
   */
  ros::Subscriber collision_checker_subscriber_;
  /**
   * Callback function for collision checker subscriber
   *
   * @param msg empty
   */
  void collisionCheckerCallback(const std_msgs::Empty::ConstPtr& msg);
  /**
   * Flag for enabling disabling collision checking
   */
  bool check_collisions_;
  /**
   * Pointer to the motion recorder
   */
  MotionRecorderPtr motion_recorder_;
  /**
   * Subscriber for turning on and off the motion recorder
   */
  ros::Subscriber motion_recorder_subscriber_;
  /**
   * Callback function for the motion recorder subscriber
   *
   * @param msg empty
   */
  void motionRecorderCallback(const std_msgs::Empty::ConstPtr& msg);
  /**
   * Flag for indicating if motion recording is in progress
   */
  bool record_motion_;
  /**
   * Subscriber for turning on and off output processing
   */
  ros::Subscriber output_control_subscriber_;
  /**
   * Callback function for turning on/off processing the output of motion retargeting
   *
   * @param msg empty
   */
  void outputControlCallback(const std_msgs::Empty::ConstPtr& msg);
  /**
   * Flag for indicating if motion recording is in progress
   */
  bool process_output_;
};

typedef boost::shared_ptr<MotionRetargeting> MotionRetargetingPtr;

} // namespace motion_retargeting

#endif /* MOTION_RETARGETING_MOTION_RETARGETING_H_ */
