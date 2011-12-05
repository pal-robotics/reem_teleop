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
 *
 * \author: Marcus Liebhardt
 *
 * This class has been inspired by Sachin Chitta's pr2_arm_kinematics and David Lu's gerneric arm_kinematics package.
 */

#include <vector>
#include <string>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/treefksolver.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <tree_kinematics/treeiksolverpos_online.hpp>

#include <kinematics_msgs/KinematicSolverInfo.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <tree_kinematics/get_tree_position_ik.h>
#include <kinematics_msgs/GetPositionFK.h>


namespace tree_kinematics
{

/*
 * \brief ...
 * ...
 *
 */
class TreeKinematics
{
  public:
    TreeKinematics(): nh_private_("~")
    {
      loop_count_ = 1;
      ik_srv_duration_ = 0.0;
      ik_srv_duration_median_ = 0.0;
      ik_duration_ = 0.0;
      ik_duration_median_ = 0.0;
    };

    ~TreeKinematics(){};

    /*
     * \brief Configures the tree_kinematics class
     *
     * This methods loads a lot of parameters from the parameter server. Most of them have hard-coded default
     * parameters. These are used, when the specific parameter could be found on the parameter server.
     * Furthermore services get registered, the robot model is created and its parameters read
     * (i.e. joint position limits) to configure the utilized kinematics solver.
     * Note: The other methods will not work properly, if this function has not been called first!
     *
     * @ return false, if critical parameters could not be found, true otherwise
     */
    bool init();

    /*
     * \brief Uses the given joint positions to calculate the position(s) of the specified end point(s)
     * in Cartesian coordinates
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
    bool getPositionFk(kinematics_msgs::GetPositionFK::Request& request,
                       kinematics_msgs::GetPositionFK::Response& response);

    /*
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
    bool getPositionIk(tree_kinematics::get_tree_position_ik::Request &request,
                       tree_kinematics::get_tree_position_ik::Response &response);

  private:
    ros::NodeHandle nh_, nh_private_;
    KDL::Tree kdl_tree_;                // KDL representation of a tree
    std::string tree_root_name_;        // The name of the tree's root
    unsigned int nr_of_jnts_;           // number of joints in the KDL::Tree
    int srv_call_frequency_;            // how often the service is called per second
    KDL::MatrixXd     js_w_matr_;       // matrix of joint weights for the IK velocity solver
    KDL::MatrixXd     ts_w_matr_;       // matrix for task space weights for the IK velocity solver
    double            lambda_;          // damping factor for the IK velocity solver

    boost::scoped_ptr<KDL::TreeFkSolverPos>         fk_solver_;
    boost::scoped_ptr<KDL::TreeIkSolverVel_wdls>    ik_vel_solver_;
    boost::scoped_ptr<KDL::TreeIkSolverPos_Online>  ik_pos_solver_;

    ros::ServiceServer fk_service_, ik_service_;
    tf::TransformListener tf_listener_;
    kinematics_msgs::KinematicSolverInfo info_;

    /* \brief Initialises the robot model and KDL::Tree and calls readJoints(...)
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
    bool loadModel(const std::string xml,
                   KDL::Tree& kdl_tree,
                   std::string& tree_root_name,
                   unsigned int& nr_of_jnts,
                   KDL::JntArray& joint_min,
                   KDL::JntArray& joint_max,
                   KDL::JntArray& joint_vel_max);

    /* \brief Retrieves information about all joints in the given model
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

    /* \brief Returns the index of the joint
     *
     * Returns the index of the joint; used by getPositionFk(...) and getPositionIk (...);
     *
     * @param name name of the joint which index should be returned
     *
     * @return the index of the joint; -1, if joint could not be found
     */
    int getJointIndex(const std::string& name);

    unsigned int loop_count_;
    double ik_srv_duration_, ik_srv_duration_median_, ik_duration_, ik_duration_median_;
};

} // namespace

