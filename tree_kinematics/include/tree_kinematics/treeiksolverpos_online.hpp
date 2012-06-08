// Copyright  (C)  2011  PAL Robotics S.L.  All rights reserved.
// Copyright  (C)  2007-2008  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

// Version: 1.0
// Author: Marcus Liebhardt
// This class has been derived from the KDL::TreeIkSolverPos_NR_JL class
// by Julia Jesse, Mikael Mayer and Ruben Smits

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDLTREEIKSOLVERPOS_ONLINE_HPP
#define KDLTREEIKSOLVERPOS_ONLINE_HPP

#include <vector>
#include <string>
#include "kdl/treeiksolver.hpp"
#include "kdl/treefksolver.hpp"
#include "kdl/jntarrayvel.hpp"

namespace KDL {

/**
 * \brief An inverse position kinematics solver for tree structures, optimized for online calculation
 * Implementation of a general inverse position kinematics algorithm to calculate the position transformation from
 * Cartesian to joint space of a general KDL::Tree. This class has been derived from the TreeIkSolverPos_NR_JL class,
 * but was modified for online solving for use in real-time systems. Thus, the calculation is only done once,
 * meaning that no iteration is done, because this solver is intended to run at a high frequency.
 * It enforces velocity limits in task as well as in joint space. It also takes joint limits into account.
 *
 * @ingroup KinematicFamily
 */
class TreeIkSolverPos_Online: public TreeIkSolverPos {
public:
    /**
     * @param nr_of_jnts number of joints of the tree to calculate the joint positions for
     * @param endpoints the list of endpoints you are interested in
     * @param q_min the minimum joint positions
     * @param q_max the maximum joint positions
     * @param q_dot_max the maximum joint velocities
     * @param x_dot_trans_max the maximum translational velocity of your endpoints
     * @param x_dot_rot_max the maximum rotational velocity of your endpoints
     * @param fksolver a forward position kinematics solver
     * @param iksolver an inverse velocity kinematics solver
     *
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
                           const double low_pass_factor = 0.0,
                           const unsigned int maxiter = 100,
                           const double eps = 1e-6);

    ~TreeIkSolverPos_Online();

    virtual double CartToJnt(const JntArray& q_in, const Frames& p_in, JntArray& q_out);

    /**
     * \brief calculates the joint positions to reach the specfied endeffector poses
     * This method does the actual calculation. Inside the velocity IK solver is called, which is calculating the
     * necessary joint velocities. The velocities are then integrated. Task space and joint space velocity clamping,
     * as well as joint position clamping is applied.
     *
     * @param q_in the current joint positions
     * @param p_in the desired poses of the endpoints
     * @param q_out the calculated joint positions and velocities
     *
     * @return returns ...
     */
    double CartToJnt_it(const JntArray& q_in, const Frames& p_in, JntArray& q_out);

private:
  /**
   * \brief Checks the velocities in cartesian space and scales them, if they exceed their maximum value
   * Scales translational and rotational velocity vectors of the class member KDL::Twist twist_,
   * if at least one of both exceeds the maximum value/length.
   * Scaling is done propotional to the biggest overshoot among both velocities.
   * Returns true, if one (or more) velocity reaches its limit.
   */
  bool enforceCartVelLimits();

  void enforceCartVelLimits_it(Twist& old_twist, Twist& current_twist);

  /**
   *\brief Checks the joint velocities and scales them, if they exceed their maximum value
   * Scales the class member KDL::JntArray q_dot_, if one (or more) joint velocity exceeds the maximum value.
   * Scaling is done propotional to the biggest overshoot among all joint velocities.
   * Returns true, if one (or more) joint reaches its limit.
   */
  bool enforceJointVelLimits();

  void enforceJointVelLimits_it(JntArray& q_dot_old, JntArray& q_dot_current);

  void filter(JntArray& q_dot, JntArray& q_out, JntArray& q_out_old);

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
  double low_pass_factor_;
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
  double low_pass_adj_factor_;
  unsigned int nr_of_still_endeffectors_;
  bool small_task_space_movement_;
};

} // namespace

#endif /* KDLTREEIKSOLVERPOS_ONLINE_HPP */

