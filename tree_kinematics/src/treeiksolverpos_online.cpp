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

#include "tree_kinematics/treeiksolverpos_online.hpp"
#include <algorithm>

namespace KDL {

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
                                               const double low_pass_factor,
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
    low_pass_factor_ = low_pass_factor;
    low_pass_adj_factor_ = 0.0;
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
  assert(q_dot_.rows() == q_out.rows());

  q_out = q_in;

  // First check, if all elements in p_in are available
  for(Frames::const_iterator f_des_it=p_in.begin();f_des_it!=p_in.end();++f_des_it)
    if(frames_.find(f_des_it->first)==frames_.end())
      return -2;

  for (Frames::const_iterator f_des_it=p_in.begin();f_des_it!=p_in.end();++f_des_it)
  {
    // Get all iterators for this endpoint
    Frames::iterator f_it = frames_.find(f_des_it->first);
    Twists::iterator delta_twists_it = delta_twists_.find(f_des_it->first);

    fksolver_.JntToCart(q_out, f_it->second, f_it->first);
    twist_ = diff(f_it->second, f_des_it->second);

    // Checks, if the twist (twist_) exceeds the maximum translational and/or rotational velocity
    // And scales them, if necessary
    enforceCartVelLimits();

    delta_twists_it->second = twist_;
  }

  double res = iksolver_.CartToJnt(q_out, delta_twists_, q_dot_);

  // Checks, if joint velocities (q_dot_) exceed their maximum velocities and scales them, if necessary
  enforceJointVelLimits();

  // Integrate
  Add(q_out, q_dot_, q_out);

  // Limit joint positions
  for (unsigned int j = 0; j < q_min_.rows(); j++)
  {
    if (q_out(j) < q_min_(j))
      q_out(j) = q_min_(j);
    else if (q_out(j) > q_max_(j))
      q_out(j) = q_max_(j);
  }

  return res;
}


double TreeIkSolverPos_Online::CartToJnt_it(const JntArray& q_in, const Frames& p_in, JntArray& q_out)
{
  assert(q_out.rows() == q_in.rows());
  assert(q_out_old_.rows() == q_in.rows());  
  assert(q_dot_.rows() == q_in.rows());
  assert(q_dot_old_.rows() == q_in.rows());
  assert(q_dot_new_.rows() == q_in.rows());



  q_out = q_in;
  SetToZero(q_dot_);
  SetToZero(q_dot_old_);
  SetToZero(q_dot_new_);
  twist_ = Twist::Zero();

  // First check, if all elements in p_in are available
  unsigned int nr_of_endeffectors = 0;
  nr_of_still_endeffectors_ = 0;
  low_pass_adj_factor_ = 0.0;
  
  for(Frames::const_iterator f_des_it=p_in.begin();f_des_it!=p_in.end();++f_des_it)
  {
    if(frames_.find(f_des_it->first)==frames_.end())
      return -2;
    else
    {
      /*
      Twists::iterator old_twists_it = old_twists_.find(f_des_it->first);
      old_twists_it->second = Twist::Zero();
      Frames::iterator f_0_it = frames_0_.find(f_des_it->first);
      */
      /*
      Twists::iterator old_twists_it = old_twists_.find(f_des_it->first);
      old_twists_it->second = diff(f_old_it->second, f_des_it->second);
      if (){};
      */
      Frames::iterator f_old_it = p_in_old_.find(f_des_it->first);
      Frames::iterator f_des_pos_l_it = frames_pos_lim_.find(f_des_it->first);
      twist_ = diff(f_old_it->second, f_des_it->second);
      /*
      if(sqrt(pow(twist_.vel.x(), 2) + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2)) < x_dot_trans_min_)
      {
        f_des_pos_l_it->second.p = addDelta(f_old_it->second.p, (0.1 * x_dot_trans_max_ * twist_.vel));
        std::cout << "old position is used." << std::endl;
      }
      else
        f_des_pos_l_it->second.p = f_des_it->second.p;
     
      if(sqrt(pow(twist_.rot.x(), 2) + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2)) < x_dot_rot_min_)
      {
        f_des_pos_l_it->second.M = addDelta(f_old_it->second.M, (0.1 * x_dot_rot_max_ * twist_.rot));
        std::cout << "old orientation is used." << std::endl;        
      }
      else
        f_des_pos_l_it->second.M = f_des_it->second.M;
      */
      f_des_pos_l_it->second.p = f_des_it->second.p;
      f_des_pos_l_it->second.M = f_des_it->second.M;
      
      Frames::iterator f_des_vel_l_it = frames_vel_lim_.find(f_des_it->first);
      fksolver_.JntToCart(q_in, f_des_vel_l_it->second, f_des_it->first);
      twist_ = diff(f_des_vel_l_it->second, f_des_pos_l_it->second);      
      
      

      f_des_vel_l_it->second = addDelta(f_des_vel_l_it->second, twist_);
      nr_of_endeffectors++;
    }
  }
  if(nr_of_still_endeffectors_ == nr_of_endeffectors)
  {
    small_task_space_movement_ = true;
    
  }
  else
    small_task_space_movement_ = false;

  unsigned int k=0;
  double res = 0.0;
  while(++k <= maxiter_)
  {
    //for (Frames::const_iterator f_des_it=p_in.begin(); f_des_it!=p_in.end(); ++f_des_it)
    for (Frames::const_iterator f_des_it=frames_vel_lim_.begin(); f_des_it!=frames_vel_lim_.end(); ++f_des_it)
    {
      // Get all iterators for this endpoint
      Frames::iterator f_it = frames_.find(f_des_it->first);
      Twists::iterator delta_twists_it = delta_twists_.find(f_des_it->first);
      Twists::iterator old_twists_it = old_twists_.find(f_des_it->first);
      Frames::iterator f_0_it = frames_vel_lim_.find(f_des_it->first);

      fksolver_.JntToCart(q_out, f_it->second, f_it->first);

      // Checks, if the current overall twist exceeds the maximum translational and/or rotational velocity.
      // If so, the velocities of the overall twist get scaled and a new current twist is calculated.
      delta_twists_it->second = diff(f_it->second, f_des_it->second);
      

      old_twists_it->second = diff(f_0_it->second, f_it->second);

      enforceCartVelLimits_it(old_twists_it->second, delta_twists_it->second);

      
    }

    res = iksolver_.CartToJnt(q_out, delta_twists_, q_dot_);

    if (res < eps_)
    {
      break;
      //return res;
    }

    // Checks, if joint velocities (q_dot_) exceed their maximum velocities and scales them, if necessary
    //Subtract(q_out, q_in, q_dot_old_);
    //enforceJointVelLimits_it(q_dot_old_, q_dot_);
    
    Subtract(q_out, q_in, q_dot_old_);

    Add(q_dot_old_, q_dot_, q_dot_);

    enforceJointVelLimits();

    Subtract(q_dot_, q_dot_old_, q_dot_);
   
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

  Subtract(q_out, q_in, q_dot_);
  
  
  Add(q_in, q_dot_, q_out);
  filter(q_dot_, q_out, q_out_old_);
  
  
  
  q_out_old_ = q_out;
  p_in_old_ = p_in;


  if (k <= maxiter_)
    return res;
  else
    return -3;
}


bool TreeIkSolverPos_Online::enforceCartVelLimits()
{
  // relative overshoot
  double rel_os_trans = 0.0;
  double rel_os_rot = 0.0;
  // biggest relative overshoot
  double rel_os_max = 0.0;
  bool max_exceeded = false;
  double x_dot_trans, x_dot_rot;
  x_dot_trans = sqrt( pow(twist_.vel.x(), 2) + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2));
  x_dot_rot = sqrt( pow(twist_.rot.x(), 2) + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2));
  
  if (x_dot_trans <= x_dot_trans_min_ && x_dot_rot <= x_dot_rot_min_ && x_dot_trans_min_ != 0.0 && x_dot_rot_min_ != 0.0)
  {
    double trans_low_pass_factor = 0.1 + 0.9 * (x_dot_trans / x_dot_trans_min_);
    double rot_low_pass_factor = 0.1 + 0.9 * (x_dot_rot / x_dot_rot_min_);

    if(low_pass_adj_factor_ < max(trans_low_pass_factor, rot_low_pass_factor))
      low_pass_adj_factor_ = max(trans_low_pass_factor, rot_low_pass_factor);

   
    nr_of_still_endeffectors_++;
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
    return false;
}


bool TreeIkSolverPos_Online::enforceJointVelLimits()
{
  // check, if one (or more) joint velocities exceed the maximum value
  // and if so, safe the biggest overshoot for scaling q_dot_ properly
  // to keep the direction of the velocity vector the same

  // relative overshoot
  double rel_os = 0.0;
  // biggest relative overshoot
  double rel_os_max = 0.0;
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
      q_dot_(i) = 0.0;
  }

  // scales q_dot_, if one joint exceeds the maximum value
  if ( max_exceeded == true )
  {
    Multiply(q_dot_, ( 1.0 / ( 1.0 + rel_os_max ) ), q_dot_);
    return true;
  }
  else
    return false;
}


void TreeIkSolverPos_Online::enforceCartVelLimits_it(Twist& old_twist, Twist& current_twist)
{
  bool max_exceeded = false;
  double x_dot_trans, x_dot_rot;

  twist_ = old_twist + current_twist;

  x_dot_trans = sqrt( pow(twist_.vel.x(), 2) + pow(twist_.vel.y(), 2) + pow(twist_.vel.z(), 2));
  x_dot_rot = sqrt( pow(twist_.rot.x(), 2) + pow(twist_.rot.y(), 2) + pow(twist_.rot.z(), 2));

  if ( x_dot_trans > x_dot_trans_max_ || x_dot_rot > x_dot_rot_max_ )
  {
    if ( x_dot_trans > x_dot_rot )
    {
      twist_.vel = twist_.vel * ( x_dot_trans_max_ / x_dot_trans );
      twist_.rot = twist_.rot * ( x_dot_trans_max_ / x_dot_trans );
      max_exceeded = true;
    }
    else if ( x_dot_rot > x_dot_trans )
    {
      twist_.vel = twist_.vel * ( x_dot_rot_max_ / x_dot_rot );
      twist_.rot = twist_.rot * ( x_dot_rot_max_ / x_dot_rot );
      max_exceeded = true;
    }
  }

  if ( max_exceeded == true )
    current_twist = twist_ - old_twist;

  //old_twist = old_twist + current_twist;
}


void TreeIkSolverPos_Online::enforceJointVelLimits_it(JntArray& q_dot_old, JntArray& q_dot_current)
{
  // check, if one (or more) joint velocities exceed the maximum value
  // and if so, safe the biggest overshoot for scaling q_dot_ properly
  // to keep the direction of the velocity vector the same
  double rel_os = 0.0; // relative overshoot
  double rel_os_max = 0.0; // biggest relative overshoot
  bool max_exceeded = false;

  // current overall joint velocities
  Add(q_dot_old, q_dot_current, q_dot_new_);

  for (unsigned int i = 0; i < q_dot_new_.rows(); ++i)
  {
    if ( q_dot_new_(i) > q_dot_max_(i) )
    {
      max_exceeded = true;
      rel_os = (q_dot_new_(i) - q_dot_max_(i)) / q_dot_max_(i);
      if ( rel_os > rel_os_max )
      {
        rel_os_max = rel_os;
      }
    }
    else if ( q_dot_new_(i) < -q_dot_max_(i) )
    {
      max_exceeded = true;
      rel_os = (-q_dot_new_(i) - q_dot_max_(i)) / q_dot_max_(i);
      if ( rel_os > rel_os_max)
      {
        rel_os_max = rel_os;
      }
    }
  }

  // scales q_dot_, if one joint exceeds the maximum value
  if ( max_exceeded == true )
  {
    Multiply(q_dot_new_, ( 1.0 / ( 1.0 + rel_os_max ) ), q_dot_new_);
    // the scaled current velocities (because q_dot_new_ is scaled)
    Subtract(q_dot_new_, q_dot_old, q_dot_current);
  }
}


void TreeIkSolverPos_Online::filter(JntArray& q_dot, JntArray& q_out, JntArray& q_out_old)
{
  // deadband filter
  //bool min_exceeded = false;
  //bool low_min_exceeded = false;
  double low_pass_factor = 1.0;
  /*
  for (unsigned int i = 0; i < q_dot.rows(); ++i)
  {
    if (q_dot(i) > q_dot_min_(i) || -q_dot(i) > q_dot_min_(i))
    {
      min_exceeded = true;
    }
    else if (q_dot(i) > (q_dot_min_(i) * 0.3) || -q_dot(i) > (q_dot_min_(i) * 0.3))
    {q
    
      low_min_exceeded = true;      
    }
  }
 
  if(!low_min_exceeded)
    low_pass_factor = 0.3 * low_pass_factor_;
  else if(!min_exceeded)
    low_pass_factor = 0.7 * low_pass_factor_;
  else
    low_pass_factor = 1.0 * low_pass_factor_;
  */

  if(small_task_space_movement_)
  {
    low_pass_factor = low_pass_adj_factor_ * low_pass_factor_;
    small_task_space_movement_ = false;
  }
  else
    low_pass_factor = low_pass_factor_;
  
  for (unsigned int i = 0; i < q_dot.rows(); ++i)
  {
    q_out(i) = low_pass_factor * q_out(i) + (1.0 - low_pass_factor) * q_out_old(i);
  }
}

} // namespace

