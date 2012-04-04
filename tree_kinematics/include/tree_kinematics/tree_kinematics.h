/*
 * Filename: reem_kinematics.h
 * Author: Marcus Liebhardt
 * Created: 15.03.2011
 * Description:
 */

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


// general things
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>

// KDL stuff
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
//#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// IK solvers
#include <kdl/treefksolver.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <tree_kinematics/treeiksolverpos_online.hpp>
//#include <tree_kinematics/treejnttojacsolver_patched.hpp>
//#include <tree_kinematics/treefksolverpos_recursive_patched.hpp>
//#include <tree_kinematics/treeiksolvervel_wdls_patched.hpp>
//#include <tree_kinematics/treeiksolverpos_nr_jl_patched.hpp>

// for self-collision checking
#include <boost/scoped_ptr.hpp>
//#include <arm_navigation_msgs/GetStateValidity.h>

// messages
#include <kinematics_msgs/KinematicSolverInfo.h>
//#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <tree_kinematics/get_tree_position_ik.h>
#include <kinematics_msgs/GetPositionFK.h>



namespace tree_kinematics{

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
    
    bool init();
    
    bool getPositionFk(kinematics_msgs::GetPositionFK::Request &request, 
                       kinematics_msgs::GetPositionFK::Response &response);
    
    bool getPositionIk(tree_kinematics::get_tree_position_ik::Request &request, 
                       tree_kinematics::get_tree_position_ik::Response &response);

  private:
    // node handles
    // private node handle is using the nodes namespace, used for checking for configuration parameters 
    // and to advertise services
    ros::NodeHandle nh_, nh_private_; 
    KDL::Tree kdl_tree_;                // KDL representation of a tree
    std::string tree_root_name_;        // The name of the root of the tree
    unsigned int nr_of_jnts_;           // number of joints in the KDL::Tree    
    int srv_call_frequency_;            // how often the service call is called
    KDL::MatrixXd     js_w_matr_;       // Matrix of joint weights for the IK velocity solver
    KDL::MatrixXd     ts_w_matr_;       // Matrix for task space weights for the IK velocity solver
    double            lambda_;          // Damping factor for the IK velocity solver

    //boost::scoped_ptr<KDL::TreeJntToJacSolver_patched>   jac_solver_; - maybe not be needed
    boost::scoped_ptr<KDL::TreeFkSolverPos>         fk_solver_;
    //boost::scoped_ptr<KDL::TreeIkSolverVel_wdls_patched>  ik_vel_solver_;
    boost::scoped_ptr<KDL::TreeIkSolverVel_wdls>    ik_vel_solver_;
    boost::scoped_ptr<KDL::TreeIkSolverPos_Online>  ik_pos_solver_;
    //boost::scoped_ptr<KDL::TreeIkSolverPos_NR_JL_patched> ik_pos_solver_;

    //ros::ServiceServer fk_solver_info_service, ik_solver_info_service;;
    ros::ServiceServer fk_service_, ik_service_;

    tf::TransformListener tf_listener_;
    
    kinematics_msgs::KinematicSolverInfo info_;
    
    bool loadModel(const std::string xml,
                   KDL::Tree &kdl_tree,
                   std::string &tree_root_name,
                   unsigned int &nr_of_jnts,
                   KDL::JntArray &joint_min, 
                   KDL::JntArray &joint_max,
                   KDL::JntArray &joint_vel_max);
                   
    bool readJoints(urdf::Model &robot_model,
                    KDL::Tree &kdl_tree,
                    std::string &tree_root_name,                                
                    unsigned int &nr_of_jnts,
                    KDL::JntArray &joint_min, 
                    KDL::JntArray &joint_max,
                    KDL::JntArray &joint_vel_max);
                                
    int getJointIndex(const std::string &name);
    
    // some stuff for performance testing
    unsigned int loop_count_;
    double ik_srv_duration_, ik_srv_duration_median_, ik_duration_, ik_duration_median_;
};

} // namespace

