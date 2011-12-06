/*
 * Software License Agreement (GNU Lesser General Public License)
 *
 * Copyright  (C)  2011  PAL Robotics S.L.  All rights reserved.
 * Copyright  (C)  2007-2008  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
 * Copyright  (C)  2008  Mikael Mayer
 * Copyright  (C)  2008  Julia Jesse
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * \author: Marcus Liebhardt
 *
 * This class has been derived from the KDL::TreeIkSolverPos_NR_JL class
 * by Julia Jesse, Mikael Mayer and Ruben Smits
 *
 */

#ifndef KDLTREEIKSOLVERPOS_ONLINE_HPP
#define KDLTREEIKSOLVERPOS_ONLINE_HPP

#include <vector>
#include <string>
#include "kdl/treeiksolver.hpp"
#include "kdl/treefksolver.hpp"
#include "kdl/jntarrayvel.hpp"

namespace KDL
{

/*
 * \brief An inverse position kinematics solver for tree structures, which additionally respects several constraints
 *
 * Implementation of a general inverse position kinematics algorithm to calculate the position transformation from
 * Cartesian to joint space of a general KDL::Tree. This class has been derived from the TreeIkSolverPos_NR_JL class.
 * Additional to enforcing joint position limits, the solver has been extended to enforce joint velocity and
 * task space velocity limits. Furthermore exponential smoothing is applied, when the end effectors stand still.
 *
 * @ingroup KinematicFamily
 */
class TreeIkSolverPos_Online: public TreeIkSolverPos
{
public:
    /*
     * @param nr_of_jnts pointer to the number of joints of the tree to calculate the joint positions for
     * @param endpoints pointer to the list of end points you are interested in
     * @param fksolver pointer to a forward position kinematics solver
     * @param iksolver pointer to an inverse velocity kinematics solver
     * @param q_min pointer to the minimum joint positions
     * @param q_max pointer to the maximum joint positions
     * @param q_dot_min pointer to the minimum joint velocities
     * @param q_dot_max pointer to the maximum joint velocities
     * @param x_dot_trans_max the maximum translational velocity of the end points
     * @param x_dot_rot_max the maximum rotational velocity of the end points
     * @param x_dot_trans_min the minimum translational velocity of the end points (default= 0.0)
     * @param x_dot_rot_min the minimum rotational velocity of the end points (default= 0.0)
     * @param smoothing_factor the smoothing factor for exponential smoothing
     * @param maxiter the maximum number of iterations
     * @param eps the precision of the solution the solver tries to reach
     */
    TreeIkSolverPos_Online(const double& nr_of_jnts,
                           const std::vector<std::string>& endpoints,
                           TreeFkSolverPos& fksolver,
                           TreeIkSolverVel& iksolver,
                           const JntArray& q_min,
                           const JntArray& q_max,
                           const JntArray& q_dot_min,
                           const JntArray& q_dot_max,
                           const double x_dot_trans_max,
                           const double x_dot_rot_max,
                           const double x_dot_trans_min = 0.0,
                           const double x_dot_rot_min = 0.0,
                           const double smoothing_factor = 0.0,
                           const unsigned int maxiter = 100,
                           const double eps = 1e-6);

    ~TreeIkSolverPos_Online();

    /*
     * \brief Calculates the joint positions to reach the specified end effector poses
     *
     * This method does the actual calculation. Inside the velocity IK solver is called, which is calculating the
     * necessary joint velocities. The velocities are then integrated. Task space and joint space velocity limiting,
     * joint position limiting and exponential smoothing is applied, if necessary.
     *
     * @param q_in the current joint positions
     * @param p_in the desired poses of the endpoints
     * @param q_out the calculated joint positions and velocities
     *
     * @return the weighted norm of the distance of the end effectors to their target poses or in case something
     * went wrong: -1 if q_in has the wrong size, -2 if one or more twists or frames could not be found,
     * -3 if the solver has reached its maximum of iterations
     */
    double CartToJnt(const JntArray& q_in, const Frames& p_in, JntArray& q_out);

private:
  /*
   * \brief Checks the velocities in Cartesian space and scales them, if they exceed their maximum value
   *
   * Scales translational and rotational velocity vectors of the class member KDL::Twist twist_,
   * if at least one of both exceeds the maximum value/length.
   * Scaling is done proportional to the biggest overshoot among both velocities.
   *
   * @return true, if one (or more) velocity reaches its limit.
   */
  bool enforceCartVelLimits();

  /*
   *\brief Checks the joint velocities and scales them, if they exceed their maximum value
   *
   * Scales the class member KDL::JntArray q_dot_, if one (or more) joint velocity exceeds the maximum value.
   * Scaling is done proportional to the biggest overshoot among all joint velocities.
   *
   * @return true, if one (or more) joint reaches its limit.
   */
  bool enforceJointVelLimits();

  /*
   * \brief Filters the joint positions using exponential smoothing
   *
   * This method filters the new joint positions using exponential smoothing and based on the smoothing factor
   * and the old joint positions.
   *
   * @param q_out the new joint positions, which shall be filtered
   * @param q_out_old old joint positions
   */
  void filter(JntArray& q_out, JntArray& q_out_old);

  TreeFkSolverPos& fksolver_;
  TreeIkSolverVel& iksolver_;
  JntArray q_min_;
  JntArray q_max_;
  JntArray q_dot_min_;
  JntArray q_dot_max_;
  double x_dot_trans_max_;
  double x_dot_rot_max_;
  double x_dot_trans_min_;
  double x_dot_rot_min_;
  double smoothing_factor_;
  unsigned int maxiter_;
  double eps_;

  JntArray q_dot_;
  Twist twist_;
  Frames frames_;
  Twists delta_twists_;
  Twists old_twists_;
  Frames frames_pos_lim_;
  Frames frames_vel_lim_;
  JntArray q_dot_old_;
  JntArray q_dot_new_;
  Frames p_in_old_;
  JntArray q_out_old_;
  double adj_smoothing_factor_;
  unsigned int nr_of_still_endeffectors_;
  bool small_task_space_movement_;
};

} // namespace

#endif /* KDLTREEIKSOLVERPOS_ONLINE_HPP */

