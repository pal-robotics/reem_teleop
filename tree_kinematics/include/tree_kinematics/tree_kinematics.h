/*
 * Software License Agreement (GNU Lesser General Public License)
 *
 * Copyright (c) 2011, PAL Robotics, S.L.
 * All rights reserved.
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
 * \author This class has been inspired by Sachin Chitta's pr2_arm_kinematics and David Lu's gerneric a
 *         arm_kinematics package.
 *
 * \copyright LPGL
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TREE_KINEMATICS_H_
#define TREE_KINEMATICS_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>

#include "tree_kinematics/kinematics_parameters.h"
#include "tree_kinematics/treeiksolverpos_online.hpp"
#include "tree_kinematics/GetTreePositionIK.h"
#include "tree_kinematics/GetPositionFK.h"
#include "tree_kinematics/KinematicSolverInfo.h"

namespace tree_kinematics
{

/**
 * \brief This class offers services for forward and inverse kinematics calculations for tree kinematic structures.
 *
 * This class is basically a wrapper around KDL's solvers for forward and inverse kinematics specialised on
 * tree kinematic structures. Once configured, services are offered for conveniently executing forward
 * and inverse kinematics calculations. This class follows the structure of the pr2_arm_kinematics and
 * the generic arm_kinematics package, which also offer similar services although for kinematic chains.
 */
class TreeKinematics
{
public:
  /**
   * \brief Configures the tree_kinematics class
   *
   * This methods loads a lot of parameters from the parameter server. Most of them have hard-coded default
   * parameters. These are used, when the specific parameter could be found on the parameter server.
   * Furthermore services get registered, the robot model is created and its parameters read
   * (i.e. joint position limits) to configure the utilized kinematics solver.
   */
  TreeKinematics(const KinematicsParameters& parameters, const ros::NodeHandle& nh);
  ~TreeKinematics(){};

  /**
   * \brief Uses the given joint positions to calculate the position(s) of the specified end point(s)
   *        in Cartesian coordinates
   *
   * This method is basically a wrapper for KDL's FK solver to get the end point position in Cartesian coordinates
   * based on the given joint positions, which are retrieved from the model.
   *
   * @ param request contains the the name of the joints and the names of end points for which the position shall be
   * calculated for
   * @ param response contains the position(s) of the specified end point(s) in Cartesian coordinates
   *
   * @ return false, if no index could be found for a given joint name or if the transformation from
   * pose message to KDL::Frame threw a exception; true otherwise
   */
  bool getPositionFk(GetPositionFK::Request& request,
                       GetPositionFK::Response& response);
  /**
   * \brief Uses the given end point position(s) in Cartesian coordinates to calculate new joint positions.
   *
   * This method is basically a wrapper for KDL's IK solver to calculated the necessary joint positions to reach
   * the given end point position(s).
   *
   * @ param request contains the current joint positions and the name(s) and desired position(s) of the end point(s)
   * @ param response contains the new joint positions as well as average joint velocities to reach
   * these new joint positions in the time given by the service call frequency
   *
   * @ return false, if no index could be found for a given joint name or if the transformation from
   * pose message to KDL::Frame threw a exception; true otherwise
   */
  bool getPositionIk(GetTreePositionIK::Request &request,
                       GetTreePositionIK::Response &response);

private:
  /**
   * The FK position solver
   */
  boost::scoped_ptr<KDL::TreeFkSolverPos> fk_solver_;
  /**
   * The IK position solver
   */
  boost::scoped_ptr<KDL::TreeIkSolverPosOnline> ik_pos_solver_;
  /**
   * The IK velocity solver - used by the IK position solver
   */
  boost::scoped_ptr<KDL::TreeIkSolverVel_wdls> ik_vel_solver_;
  /**
   * Some information about the kinematics solver
   */
  tree_kinematics::KinematicSolverInfo info_;
  /**
   * Parameters for configuring tree kinematics
   */
  KinematicsParameters parameters_;
  /**
   * Node handles for ? TODO
   */
  ros::NodeHandle nh_, nh_private_;
  /**
   * KDL representation of kinematics with tree structures
   */
  KDL::Tree kdl_tree_;
  /**
   * The name of the kinematics tree's root
   */
  std::string tree_root_name_;
  /**
   * Number of joints in the KDL::Tree
   */
  unsigned int nr_of_jnts_;
  /**
   * TF listener for retrieving tf transforms
   */
  tf::TransformListener tf_listener_;
  /**
   * Used for time measurements
   */
  double ik_srv_duration_, ik_srv_duration_median_, ik_duration_, ik_duration_median_;
  /**
   * Used for time measurements
   */
  unsigned int loop_count_;

  /**
   * \brief Initialises the robot model and KDL::Tree and calls readJoints(...)
   *
   * This method initialises the robot model and KDL::Tree and calls readJoints(...).
   *
   * @param xml the xml description of the robot model
   * @param kdl_tree pointer to the KDL::Tree
   * @param tree_root_name pointer to the root name, which defines the (sub)tree
   * @param nr_of_jnts pointer to the number of joints
   * @param joint_min pointer to an array containing the minimum position limits of the joints
   * @param joint_max pointer to an array containing the maximum position limits of the joints
   * @param joint_vel_max pointer to an array containing the maximum absolute velocity limits of the joints
   *
   * @return false, if the robot model or KDL::Tree could not be initialised or readJoint(...) returned false,
   * true otherwise
   */
  bool parseModelDescription(const std::string xml,
                                KDL::Tree& kdl_tree,
                                std::string& tree_root_name,
                                unsigned int& nr_of_jnts,
                                KDL::JntArray& joint_min,
                                KDL::JntArray& joint_max,
                                KDL::JntArray& joint_vel_max);

  /**
   * \brief Retrieves information about all joints in the given model
   *
   * Using the provided root a subtree is extracted. This subtree is then searched for the all joints and their
   * position and velocity limits.
   * This method uses readJoints(...).
   *
   * @param xml pointer to the robot's model containing the desired joint information
   * @param kdl_tree pointer to the KDL::Tree
   * @param tree_root_name pointer to the root name, which defines the (sub)tree
   * @param nr_of_jnts pointer to the number of joints
   * @param joint_min pointer to an array containing the minimum position limits of the joints
   * @param joint_max pointer to an array containing the maximum position limits of the joints
   * @param joint_vel_max pointer to an array containing the maximum absolute velocity limits of the joints
   *
   * @return false, if a joint of the kdl tree could not be found in the robot model, true otherwise
   */
  bool readJoints(urdf::Model& robot_model,
                    KDL::Tree& kdl_tree,
                    std::string& tree_root_name,
                    unsigned int& nr_of_jnts,
                    KDL::JntArray& joint_min,
                    KDL::JntArray& joint_max,
                    KDL::JntArray& joint_vel_max);

  /**
   * \brief Returns the index of the joint
   *
   * Returns the index of the joint; used by getPositionFk(...) and getPositionIk (...);
   *
   * @param name name of the joint which index should be returned
   *
   * @return the index of the joint; -1, if joint could not be found
   */
  int getJointIndex(const std::string& name);
};

/**
 * Convenience typedef for a boost shared pointer to the TreeKinematics class
 */
typedef boost::shared_ptr<TreeKinematics> TreeKinematicsPtr;

} // namespace

#endif /* TREE_KINEMATICS_H_ */
