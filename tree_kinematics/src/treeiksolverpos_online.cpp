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
 */
/**
 * \author Marcus Liebhardt
 * \author This class has been derived from the KDL::TreeIkSolverPos_NR_JL class
 *         by Julia Jesse, Mikael Mayer and Ruben Smits.
 * \copyright LPGL
 */

#include "tree_kinematics/treeiksolverpos_online.hpp"
#include <algorithm>

namespace KDL
{

TreeIkSolverPos_Online::TreeIkSolverPos_Online(const double& nr_of_jnts,
                                               const std::vector<std::string>& endpoints,
                                               TreeFkSolverPos& fksolver,
                                               TreeIkSolverVel& iksolver,
                                               const JntArray& q_min,
                                               const JntArray& q_max,
                                               const JntArray& q_dot_min,
                                               const JntArray& q_dot_max,
                                               const double x_dot_trans_max,
                                               const double x_dot_rot_max,
                                               const double x_dot_trans_min,
                                               const double x_dot_rot_min,
                                               const double smoothing_factor,
                                               const unsigned int maxiter,
                                               const double eps) :
                                               fksolver_(fksolver),
                                               iksolver_(iksolver),
                                               q_min_(nr_of_jnts),
                                               q_max_(nr_of_jnts),
                                               q_dot_min_(nr_of_jnts),
                                               q_dot_max_(nr_of_jnts),
                                               maxiter_(maxiter),
                                               eps_(eps),
                                               q_dot_(nr_of_jnts),
                                               q_dot_old_(nr_of_jnts),
                                               q_dot_new_(nr_of_jnts),
                                               q_out_old_(nr_of_jnts)
{
  assert(q_min.rows() == nr_of_jnts);
  assert(q_max.rows() == nr_of_jnts);
  assert(q_dot_max.rows() == nr_of_jnts);
  assert(q_out_old_.rows() == nr_of_jnts);

  SetToZero(q_out_old_);
  q_min_ = q_min;
  q_max_ = q_max;
  q_dot_min_ = q_dot_min;
  q_dot_max_ = q_dot_max;
  x_dot_trans_max_ = x_dot_trans_max;
  x_dot_rot_max_ = x_dot_rot_max;
  x_dot_trans_min_ = x_dot_trans_min;
  x_dot_rot_min_ = x_dot_rot_min;
  smoothing_factor_ = smoothing_factor;
  adj_smoothing_factor_ = 0.0;
  nr_of_still_endeffectors_ = 0;
  small_task_space_movement_ = false;

  for (size_t i = 0; i < endpoints.size(); i++)
  {

    frames_.insert(Frames::value_type(endpoints[i], Frame::Identity()));
    delta_twists_.insert(Twists::value_type(endpoints[i], Twist::Zero()));
    old_twists_.insert(Twists::value_type(endpoints[i], Twist::Zero()));
    frames_pos_lim_.insert(Frames::value_type(endpoints[i], Frame::Identity()));
    frames_vel_lim_.insert(Frames::value_type(endpoints[i], Frame::Identity()));
    p_in_old_.insert(Frames::value_type(endpoints[i], Frame::Identity()));

  }
}


TreeIkSolverPos_Online::~TreeIkSolverPos_Online()
{}


double TreeIkSolverPos_Online::CartToJnt(const JntArray& q_in, const Frames& p_in, JntArray& q_out)
{
  assert(q_out.rows() == q_in.rows());
  assert(q_out_old_.rows() == q_in.rows());
  assert(q_dot_.rows() == q_in.rows());
  assert(q_dot_old_.rows() == q_in.rows());
  assert(q_dot_new_.rows() == q_in.rows());
  /*
  std::cout << "--- IK call started ---" << std::endl;
  */
  q_out = q_in;
  SetToZero(q_dot_);
  SetToZero(q_dot_old_);
  SetToZero(q_dot_new_);
  twist_ = Twist::Zero();

  unsigned int nr_of_endeffectors = 0;
  nr_of_still_endeffectors_ = 0;
  adj_smoothing_factor_ = 0.0;
  for(Frames::const_iterator f_des_it=p_in.begin();f_des_it!=p_in.end();++f_des_it)
  {
    // First check, if all elements in p_in are available
    if(frames_.find(f_des_it->first)==frames_.end())
      return -2;
    // if so limit the Cartesian velocities if necessary and
    // check if the end effectors can be considered as holding still
    else
    {
      Frames::iterator f_des_pos_l_it = frames_pos_lim_.find(f_des_it->first);
      f_des_pos_l_it->second.p = f_des_it->second.p;
      f_des_pos_l_it->second.M = f_des_it->second.M;
      Frames::iterator f_des_vel_l_it = frames_vel_lim_.find(f_des_it->first);
      fksolver_.JntToCart(q_in, f_des_vel_l_it->second, f_des_it->first);
      twist_ = diff(f_des_vel_l_it->second, f_des_pos_l_it->second);
      /*
      std::cout << "desired twist.vel (x_d - x_c) " << sqrt( pow(twist_.vel.x(), 2)
      + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2)) << std::endl;
      std::cout << "desired twist.rot (x_d - x_c) " << sqrt( pow(twist_.rot.x(), 2)
      + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2)) << std::endl;
      */
      enforceCartVelLimits();
      /*
      std::cout << "limitted twist.vel (x_d - x_c) " << sqrt( pow(twist_.vel.x(), 2)
      + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2)) << std::endl;
      std::cout << "limitted twist.rot (x_d - x_c) " << sqrt( pow(twist_.rot.x(), 2)
      + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2)) << std::endl;
      */
      f_des_vel_l_it->second = addDelta(f_des_vel_l_it->second, twist_);
      nr_of_endeffectors++;
    }
  }
  if(nr_of_still_endeffectors_ == nr_of_endeffectors)
  {
    small_task_space_movement_ = true;
    /*
    std::cout << "Task space movements of all endeffectors are small!" << std::endl;
    std::cout << "Given smoothing factor will be adjusted by ";
    std::cout << adj_smoothing_factor_ << " (in total = " << adj_smoothing_factor_ * smoothing_factor_ << ")"
    << std::endl;
    */
  }
  else
    small_task_space_movement_ = false;

  unsigned int k=0;
  double res = 0.0;
  while(++k <= maxiter_)
  {
    for (Frames::const_iterator f_des_it=frames_vel_lim_.begin(); f_des_it!=frames_vel_lim_.end(); ++f_des_it)
    {
      // Get all iterators for this endpoint
      Frames::iterator f_it = frames_.find(f_des_it->first);
      Twists::iterator delta_twists_it = delta_twists_.find(f_des_it->first);
      Twists::iterator old_twists_it = old_twists_.find(f_des_it->first);
      Frames::iterator f_0_it = frames_vel_lim_.find(f_des_it->first);
      // Calculate forward kinematics with the current joint position
      fksolver_.JntToCart(q_out, f_it->second, f_it->first);
      // Calculate difference between current and target end point positions (in Cartesian coordinates)
      delta_twists_it->second = diff(f_it->second, f_des_it->second);
    }
    // Calculate joint velocities
    res = iksolver_.CartToJnt(q_out, delta_twists_, q_dot_);
    // Stop iterations if solution is precise enough
    if (res < eps_)
    {
      break;
    }
    // Integrate
    Add(q_out, q_dot_, q_out);
    // Limit joint positions
    for (unsigned int j = 0; j < q_min_.rows(); ++j)
    {
      if (q_out(j) < q_min_(j))
        q_out(j) = q_min_(j);
      else if (q_out(j) > q_max_(j))
        q_out(j) = q_max_(j);
    }
  }
  // Check if joint velocities (q_dot_) exceed their maximum velocities and scale them, if necessary
  Subtract(q_out, q_in, q_dot_);
  /*
  for (unsigned int i = 0; i < q_dot_.rows(); ++i)
  {
    std::cout << "q_dot_(" << i << "): " << q_dot_(i) << std::endl;
  }
  */
  enforceJointVelLimits();
  /*
  if(enforceJointVelLimits())
    std::cout << "q_dot_ got limitted!" << std::endl;
  else
    std::cout << "q_dot_ got not limitted!" << std::endl;
  for (unsigned int i = 0; i < q_dot_.rows(); ++i)
  {
    std::cout << "limitted q_dot_(" << i << "): " << q_dot_(i) << std::endl;
  }
  */
  // Integrate again with the scaled (if necessary) joint velocities
  Add(q_in, q_dot_, q_out);
  // Filter solution
  filter(q_out, q_out_old_);
  /*
  std::cout << "q_out:" << std::endl;
  for (unsigned int i = 0; i < q_out.rows(); ++i)
  {
    std::cout << "q_out(" << i << "): " << q_out(i) << std::endl;
  }
  */
  q_out_old_ = q_out;
  p_in_old_ = p_in;
  /*
  std::cout << "--- IK call ended ---" << std::endl;
  */
  if (k <= maxiter_)
    return res;
  else
    return -3; // res already contains -1 and -2 coming from the velocity IK solver
}


bool TreeIkSolverPos_Online::enforceCartVelLimits()
{
  double rel_os_trans = 0.0; // relative translational overshoot
  double rel_os_rot = 0.0; // relative rotational overshoot
  double rel_os_max = 0.0; // biggest relative overshoot
  bool max_exceeded = false;
  double x_dot_trans, x_dot_rot;
  x_dot_trans = sqrt( pow(twist_.vel.x(), 2) + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2));
  x_dot_rot = sqrt( pow(twist_.rot.x(), 2) + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2));

  if (x_dot_trans <= x_dot_trans_min_ && x_dot_rot <= x_dot_rot_min_ && x_dot_trans_min_ != 0.0 && x_dot_rot_min_ != 0.0)
  {
    double trans_smoothing_factor = 0.1 + 0.9 * (x_dot_trans / x_dot_trans_min_);
    double rot_smoothing_factor = 0.1 + 0.9 * (x_dot_rot / x_dot_rot_min_);
    if(adj_smoothing_factor_ < max(trans_smoothing_factor, rot_smoothing_factor))
    {
      adj_smoothing_factor_ = max(trans_smoothing_factor, rot_smoothing_factor);
    }
    nr_of_still_endeffectors_++;
    /*
    std::cout << "Task space movement is small. Increasing number of still endeffectors." << std::endl;
    std::cout << "Applied smoothing_factor would be = " << adj_smoothing_factor_ * smoothing_factor_ << std::endl;
    */
  }
  if ( x_dot_trans > x_dot_trans_max_ || x_dot_rot > x_dot_rot_max_ )
  {
    rel_os_trans = (x_dot_trans - x_dot_trans_max_) / x_dot_trans_max_;
    rel_os_rot = (x_dot_rot - x_dot_rot_max_) / x_dot_rot_max_;
    if ( rel_os_trans >= rel_os_rot )
    {
      max_exceeded = true;
      rel_os_max = rel_os_trans;
    }
    else if ( rel_os_trans < rel_os_rot )
    {
      max_exceeded = true;
      rel_os_max = rel_os_rot;
    }
  }
  if ( max_exceeded == true )
  {
    twist_.vel = twist_.vel * ( 1.0 / ( 1.0 + rel_os_max ) );
    twist_.rot = twist_.rot * ( 1.0 / ( 1.0 + rel_os_max ) );
    return true;
  }
  else
  {
    return false;
  }
}


bool TreeIkSolverPos_Online::enforceJointVelLimits()
{
  double rel_os = 0.0; // relative overshoot
  double rel_os_max = 0.0; // biggest relative overshoot
  bool max_exceeded = false;
  for (unsigned int i = 0; i < q_dot_.rows(); ++i)
  {
    if ( q_dot_(i) > q_dot_max_(i) && q_dot_max_(i) != 0.0)
    {
      max_exceeded = true;
      rel_os = (q_dot_(i) - q_dot_max_(i)) / q_dot_max_(i);
      if ( rel_os > rel_os_max )
      {
        rel_os_max = rel_os;
      }
    }
    else if ( q_dot_(i) < -q_dot_max_(i) && q_dot_max_(i) != 0.0)
    {
      max_exceeded = true;
      rel_os = (-q_dot_(i) - q_dot_max_(i)) / q_dot_max_(i);
      if ( rel_os > rel_os_max)
      {
        rel_os_max = rel_os;
      }
    }
    else if (q_dot_max_(i) == 0.0)
    {
      q_dot_(i) = 0.0;
    }
  }
  // scales q_dot_, if one joint exceeds the maximum value
  if ( max_exceeded == true )
  {
    Multiply(q_dot_, ( 1.0 / ( 1.0 + rel_os_max ) ), q_dot_);
    return true;
  }
  else
  {
    return false;
  }
}


void TreeIkSolverPos_Online::filter(JntArray& q_out, JntArray& q_out_old)
{
  double smoothing_factor = 1.0;
  if(small_task_space_movement_)
  {
    smoothing_factor = adj_smoothing_factor_ * smoothing_factor_;
    small_task_space_movement_ = false;
  }
  else
  {
    smoothing_factor = smoothing_factor_;
  }
  for (unsigned int i = 0; i < q_out.rows(); ++i)
  {
    q_out(i) = smoothing_factor * q_out(i) + (1.0 - smoothing_factor) * q_out_old(i);
  }
}

} // namespace

