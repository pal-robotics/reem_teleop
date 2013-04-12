/**
 * \author Marcus Liebhardt
 *
 * \copyright BSD
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KINEMATICS_PARAMETERS_H_
#define KINEMATICS_PARAMETERS_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <string>

namespace tree_kinematics
{

/**
 * IK parameters for configuring the Tree IK solver
 */
struct KinematicsParameters
{
  /**
   * Parameter name which holds the robot's URDF model description
   */
  std::string robot_model_description_name;
  /**
   * Names of all end points involved in the FK/IK calculations
   */
  std::vector<std::string> endpt_names;
  /**
   * Maximum precision for the IK calculation
   */
  double epsilon;
  /**
   * Maximum iterations for the IK calculation
   */
  double max_iterations;
  /**
   * Damping factor for the IK velocity solver
   * Introduces small errors into the calculations for avoiding singularities
   */
  double lambda;
  /**
   * Frequency of IK calls - used for velocity limiting
   */
  int ik_call_frequency;
  /**
   * Factor used for determining the maximum joint velocity (is scaled by ik_call_frequency)
   */
  double q_dot_max_factor;
  /**
   * Factor used for determining the minimum joint velocity (is scaled by ik_call_frequency)
   */
  double q_dot_min_factor;
  /**
   * Factor used for determining the maximum translational end point velocity (is scaled by ik_call_frequency)
   */
  double x_dot_trans_max_factor;
  /**
   * Factor used for determining the minimum translational end point velocity (is scaled by ik_call_frequency)
   */
  double x_dot_trans_min_factor;
  /**
   * Factor used for determining the maximum rotational end point velocity (is scaled by ik_call_frequency)
   */
  double x_dot_rot_max_factor;
  /**
   * Factor used for determining the minimum rotational end point velocity (is scaled by ik_call_frequency)
   */
  double x_dot_rot_min_factor;
  /**
   * Factor used for low pass filtering - applied when all end points are considered resting
   */
  double low_pass_factor;
  /**
   * Matrix for task space weights for the IK velocity solver ((6 x #end points) x (6 x #end points))
   */
  std::vector< std::vector<double> > task_space_weights;
  /**
   * Matrix of joint weights for the IK velocity solver (#joints x #joints)
   */
  std::vector<double> joint_space_weights;
};

} // namespace

#endif /* KINEMATICS_PARAMETERS_H_ */
