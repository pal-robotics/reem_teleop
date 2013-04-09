#include <string>
#include <vector>

#include <ros/ros.h>
//#include <tf/StampedTransform.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <motion_adaption/motion_adaption.h>
//#include <tree_kinematics/tree_kinematics.h>

#include "motion_retargeting_configuration.h"

namespace motion_retargeting
{

class MotionRetargeting
{
public:
  /**
   * Initialies motion retargeting by configures motion adaption and tree kinematics
   */
  MotionRetargeting(MotionRetargetingConfiguration& retargeting_config);
  /**
   * Throws out the trash
   */
  ~MotionRetargeting();
  /**
   * Does a one shot retargeting of the input task space motion to the output joint space motion
   * Adaption of the whole incoming motions needs to be completed before the IK can be calculated.
   * @param output_joint_states Contains a waypoint consisting of positions and velocities for each joint
   * @return true, if now error occurred
   *
   * TODO:
   *   * Current input (retrieveing tf transforms) is handled by motion adapation. Should we outsource that
   *     and make it the responsibility of the program using this class?
   *     * Pros: more flexible
   *     * Cons: need to checking of correct input and output each time -> takes time
   *   * Current output is fixed as well. Should we make it more generic? If yes, how?
   */
  bool MotionRetargeting::retarget(/*trajectory_msgs::JointTrajectoryPoint& output_joint_states*/);

private:
  ros::NodeHandle nh_;
  motion_adaption::MotionAdaption* motion_adaption_; // input: target pose from motion tracking devices, output: adapted pose
//  TreeKinematics* tree_kinematics_; // input: adapted poses from motion adaption, output: target joint positions and velocities
};

} // namespace motion_retargeting
