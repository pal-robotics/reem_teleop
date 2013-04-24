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
 * \copyright LPGL
 */

#include <urdf_model/joint.h>
#include <kdl_conversions/kdl_msg.h>
#include "tree_kinematics/tree_kinematics.h"


namespace tree_kinematics
{

TreeKinematics::TreeKinematics(const KinematicsParameters& parameters, const ros::NodeHandle& nh) :
                                   parameters_(parameters), nh_(nh)
{
  /*
   * Debug output for incoming parameters
   */
  ROS_DEBUG_STREAM("Kinematics parameters:");
  ROS_DEBUG_STREAM("ik_call_frequency  = " << parameters_.ik_call_frequency);
  ROS_DEBUG_STREAM("maximum iterations = " << parameters_.max_iterations);
  ROS_DEBUG_STREAM("epsilon            = " << parameters_.epsilon);
  ROS_DEBUG_STREAM("lambda             = " << parameters_.lambda);
  ROS_DEBUG_STREAM("x_dot_trans_max    = " << parameters_.x_dot_trans_max_factor);
  ROS_DEBUG_STREAM("x_dot_trans_min    = " << parameters_.x_dot_trans_min_factor);
  ROS_DEBUG_STREAM("x_dot_rot_max      = " << parameters_.x_dot_rot_max_factor);
  ROS_DEBUG_STREAM("x_dot_rot_min      = " << parameters_.x_dot_rot_min_factor);
  ROS_DEBUG_STREAM("q_dot_max_factor   = " << parameters_.q_dot_max_factor);
  ROS_DEBUG_STREAM("q_dot_min_factor   = " << parameters_.q_dot_min_factor);
  ROS_DEBUG_STREAM("low_pass_factor    = " << parameters_.low_pass_factor);

  /*
   * Create the tree kinematics representation
   */
  // get URDF XML - IMPROVE!
  std::string urdf_xml, full_urdf_xml;
  nh_.param("urdf_xml_model",urdf_xml, parameters_.robot_model_description_name);
  nh_.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG_STREAM("Reading URDF model description from parameter server ...");
  std::string result;
  if (!nh_.getParam(full_urdf_xml, result))
  {
    ROS_ERROR_STREAM("Could not load the URDF model description '" << urdf_xml << "' from parameter server!");
    throw;
  }
  // data for the IK position solver obtained from the robot model
  KDL::JntArray     joint_min;             // Minimum joint positions
  KDL::JntArray     joint_max;             // Maximum joint positions
  KDL::JntArray     joint_vel_max;         // Maximum joint velocities
  // create robot model, KDL::Tree and retrieve model parameters from the model
  if (!parseModelDescription(result, kdl_tree_, tree_root_name_, nr_of_jnts_, joint_min, joint_max, joint_vel_max))
  {
    ROS_ERROR_STREAM("Could not load model!");
    throw;
  }

  /*
   * Scale limits
   */
  double x_dot_trans_max, x_dot_rot_max, x_dot_trans_min, x_dot_rot_min, q_dot_min, q_dot_max;
  x_dot_trans_max = parameters_.x_dot_trans_max_factor / parameters_.ik_call_frequency;
  x_dot_rot_max = parameters_.x_dot_trans_max_factor / parameters_.ik_call_frequency;
  x_dot_trans_min = parameters_.x_dot_trans_max_factor / parameters_.ik_call_frequency;
  x_dot_rot_min = parameters_.x_dot_trans_max_factor / parameters_.ik_call_frequency;
  q_dot_max = parameters_.q_dot_max_factor / parameters_.ik_call_frequency;
  q_dot_min = parameters_.q_dot_min_factor / parameters_.ik_call_frequency;
  KDL::Multiply(joint_vel_max,  parameters_.q_dot_max_factor, joint_vel_max);
  KDL::JntArray joint_vel_min(nr_of_jnts_); // Minimum joint velocities
  for (unsigned int i = 0; i < joint_vel_min.rows(); ++i)
  {
    joint_vel_min(i) = 1.0;
  }
  KDL::Multiply(joint_vel_min,  parameters_.q_dot_min_factor, joint_vel_min);

  /*
   * Create the kinematics solvers
   */
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(kdl_tree_));
  ik_vel_solver_.reset(new KDL::TreeIkSolverVel_wdls(kdl_tree_, parameters_.endpt_names));
  ik_pos_solver_.reset(new KDL::TreeIkSolverPos_Online(nr_of_jnts_,
                                                       parameters_.endpt_names,
                                                       *fk_solver_,
                                                       *ik_vel_solver_,
                                                       joint_min,
                                                       joint_max,
                                                       joint_vel_min,
                                                       joint_vel_max,
                                                       x_dot_trans_max,
                                                       x_dot_rot_max,
                                                       x_dot_trans_min,
                                                       x_dot_rot_min,
                                                       parameters_.low_pass_factor,
                                                       parameters_.max_iterations,
                                                       parameters_.epsilon));

  /*
   * Special configuration of the IK velocity solver
   */
  // Task space weight matrix
  ROS_DEBUG_STREAM("Configuring the task space weight matrix ...");
  if (parameters_.task_space_weights.size() != parameters_.endpt_names.size())
  {
    ROS_ERROR_STREAM("The number of task space weights does not match the number of end points! ("
                     << parameters_.task_space_weights.size() << " != " <<  parameters_.endpt_names.size() << ")");
    throw;
  }
  for (unsigned int endpt = 0 ; endpt < parameters_.endpt_names.size() ; ++endpt)
  {
    if (parameters_.task_space_weights[endpt].size() != 6)
    {
      ROS_ERROR_STREAM("The number of task space  weights for end point '" << parameters_.endpt_names[endpt]
                       << "' is incorrect!" << parameters_.task_space_weights[endpt].size() << " != 6)");
      throw;
    }
  }
  KDL::MatrixXd ts_w_matr = KDL::MatrixXd::Identity(6 * parameters_.endpt_names.size(),
                                                    6 * parameters_.endpt_names.size());
  for (unsigned int endpt = 0 ; endpt < parameters_.endpt_names.size() ; ++endpt)
  {
    for (unsigned int weight = 0 ; weight < 6; ++weight)
    {
      ts_w_matr( endpt * 6 + weight, endpt * 6 + weight ) = parameters_.task_space_weights[endpt][weight];
      ROS_DEBUG_STREAM("ts_w_matr_(" << (endpt * 6 + weight) << ", " << (endpt * 6 + weight) << "): "
                       << ts_w_matr( endpt * 6 + weight, endpt * 6 + weight ));
    }
  }
  ik_vel_solver_->setWeightTS(ts_w_matr);
  ROS_DEBUG_STREAM("Task space weight matrix configured.");

  // Joint space weight matrix
  ROS_DEBUG_STREAM("Configuring the joint space weight matrix ...");
  if (parameters_.joint_space_weights.size() != nr_of_jnts_)
  {
    ROS_ERROR_STREAM("The number of joint weights does not match the number joints! ("
                     << parameters_.joint_space_weights.size() << " != " <<  nr_of_jnts_ << ")");
    throw;
  }
  KDL::MatrixXd js_w_matr = KDL::MatrixXd::Identity(nr_of_jnts_, nr_of_jnts_);
  for (unsigned int i = 0 ; i < nr_of_jnts_ ; ++i)
  {
    js_w_matr( i, i ) = parameters_.joint_space_weights[i];
    ROS_DEBUG_STREAM("qs_w_matr(" << i << ", " << i << "): " << js_w_matr( i, i ));
  }
  ik_vel_solver_->setWeightJS(js_w_matr);
  ROS_DEBUG_STREAM("Joint space weight matrix configured.");

  // lambda
  ik_vel_solver_->setLambda(parameters_.lambda);

  /*
   * Initialise time measurement variables
   */
  loop_count_ = 1;
  ik_srv_duration_ = 0.0;
  ik_srv_duration_median_ = 0.0;
  ik_duration_ = 0.0;
  ik_duration_median_ = 0.0;
};

bool TreeKinematics::parseModelDescription(const std::string xml,
                                                KDL::Tree &kdl_tree,
                                                std::string &tree_root_name,
                                                unsigned int &nr_of_jnts,
                                                KDL::JntArray &joint_min,
                                                KDL::JntArray &joint_max,
                                                KDL::JntArray &joint_vel_max)
{
  urdf::Model robot_model;
  if (!robot_model.initString(xml))
  {
      ROS_FATAL("Could not initialize robot model!");
      return false;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
  {
      ROS_FATAL("Could not initialize tree object!");
      return false;
  }
  if (!readJoints(robot_model, kdl_tree, tree_root_name, nr_of_jnts, joint_min, joint_max, joint_vel_max))
  {
      ROS_FATAL("Could not read information about the joints!");
      return false;
  }
  return true;
}


bool TreeKinematics::readJoints(urdf::Model &robot_model,
                                KDL::Tree &kdl_tree,
                                std::string &tree_root_name,
                                unsigned int &nr_of_jnts,
                                KDL::JntArray &joint_min,
                                KDL::JntArray &joint_max,
                                KDL::JntArray &joint_vel_max)
{
  KDL::SegmentMap segmentMap;
  segmentMap = kdl_tree.getSegments();
  tree_root_name = kdl_tree.getRootSegment()->second.segment.getName();
  nr_of_jnts = kdl_tree.getNrOfJoints();
  ROS_DEBUG( "the tree's number of joints: [%d]", nr_of_jnts );
  joint_min.resize(nr_of_jnts);
  joint_max.resize(nr_of_jnts);
  joint_vel_max.resize(nr_of_jnts);
  info_.joint_names.resize(nr_of_jnts);
  info_.limits.resize(nr_of_jnts);

  // The following walks through all tree segments, extracts their joints except joints of KDL::None and extracts
  // the information about min/max position and max velocity of the joints not of type urdf::Joint::UNKNOWN or
  // urdf::Joint::FIXED
  ROS_DEBUG("Extracting all joints from the tree, which are not of type KDL::Joint::None.");
  boost::shared_ptr<const urdf::Joint> joint;
  for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end(); ++seg_it)
  {
    if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      // check, if joint can be found in the URDF model of the robot
      joint = robot_model.getJoint(seg_it->second.segment.getJoint().getName().c_str());
      if (!joint)
      {
        ROS_FATAL("Joint '%s' has not been found in the URDF robot model!", joint->name.c_str());
        return false;
      }
      // extract joint information
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_DEBUG( "getting information about joint: [%s]", joint->name.c_str() );
        double lower = 0.0, upper = 0.0, vel_limit = 0.0;
        unsigned int has_pos_limits = 0, has_vel_limits = 0;

        if ( joint->type != urdf::Joint::CONTINUOUS )
        {
          ROS_DEBUG("joint is not continuous.");
          lower = joint->limits->lower;
          upper = joint->limits->upper;
          has_pos_limits = 1;
          if (joint->limits->velocity)
          {
            has_vel_limits = 1;
            vel_limit = joint->limits->velocity;
            ROS_DEBUG("joint has following velocity limit: %f", vel_limit);
          }
          else
          {
            has_vel_limits = 0;
            vel_limit = 0.0;
            ROS_DEBUG("joint has no velocity limit.");
          }
        }
        else
        {
          ROS_DEBUG("joint is continuous.");
          lower = -M_PI;
          upper = M_PI;
          has_pos_limits = 0;
          if(joint->limits && joint->limits->velocity)
          {
            has_vel_limits = 1;
            vel_limit = joint->limits->velocity;
            ROS_DEBUG("joint has following velocity limit: %f", vel_limit);
          }
          else
          {
            has_vel_limits = 0;
            vel_limit = 0.0;
            ROS_DEBUG("joint has no velocity limit.");
          }
        }

        joint_min(seg_it->second.q_nr) = lower;
        joint_max(seg_it->second.q_nr) = upper;
        joint_vel_max(seg_it->second.q_nr) = vel_limit;
        ROS_DEBUG("pos_min = %f, pos_max = %f, vel_max = %f", lower, upper, vel_limit);

        info_.joint_names[seg_it->second.q_nr] = joint->name;
        info_.limits[seg_it->second.q_nr].joint_name = joint->name;
        info_.limits[seg_it->second.q_nr].has_position_limits = has_pos_limits;
        info_.limits[seg_it->second.q_nr].min_position = lower;
        info_.limits[seg_it->second.q_nr].max_position = upper;
        info_.limits[seg_it->second.q_nr].has_velocity_limits = has_vel_limits;
        info_.limits[seg_it->second.q_nr].max_velocity = vel_limit;
      }
    }
  }
  return true;
}


int TreeKinematics::getJointIndex(const std::string &name)
{
    for (unsigned int i=0; i < info_.joint_names.size(); ++i)
    {
      if (info_.joint_names[i] == name)
        return i;
    }
    return -1;
}


bool TreeKinematics::getPositionFk(GetPositionFK::Request& request,
                                       GetPositionFK::Response& response)
{
  ROS_DEBUG_STREAM("tree_kinematics: getPositionFK method invoked.");
  KDL::JntArray q_in;
  double nr_of_jnts = request.robot_state.name.size();
  q_in.resize(nr_of_jnts);

  for (unsigned int i=0; i < nr_of_jnts; ++i)
  {
    int tmp_index = getJointIndex(request.robot_state.name[i]);
    if (tmp_index >=0)
    {
      q_in(tmp_index) = request.robot_state.position[i];
      ROS_DEBUG_STREAM("Joint '" << request.robot_state.name[i] << "' is now number '" << tmp_index << "'.");
    }
    else
    {
      ROS_ERROR_STREAM(i << ": No index for joint '" << request.robot_state.name[i] << "'! Aborting service call.");
      return false;
    }
  }

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());
  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  for (unsigned int i=0; i < request.fk_link_names.size(); i++)
  {
    ROS_DEBUG_STREAM("End effector name: " << request.fk_link_names[i]);
    int fk_ret = fk_solver_->JntToCart(q_in, p_out, request.fk_link_names[i]);
    if (fk_ret >= 0)
    {
      pose.header.frame_id = tree_root_name_;
      pose.header.stamp = ros::Time::now();
      tf::poseKDLToMsg(p_out, pose.pose);
      response.pose_stamped[i] = pose;
      response.fk_link_names[i] = request.fk_link_names[i];
//      response.error_code.val = response.error_code.SUCCESS;
    }
    else
    {
      ROS_WARN_STREAM("A FK solution could not be found for " << request.fk_link_names[i]
                      << "(error code = " << fk_ret << ").");
//      response.error_code.val = response.error_code.NO_FK_SOLUTION;
    }
  }

  return true;
}


bool TreeKinematics::getPositionIk(tree_kinematics::GetTreePositionIK::Request &request,
                                       tree_kinematics::GetTreePositionIK::Response &response)
{
  ik_srv_duration_ = ros::Time::now().toSec();
  ROS_DEBUG_STREAM("tree kinematics: getPositionIK method invoked.");
  if (request.endpt_names.size() != request.endpt_poses.size())
  {
    ROS_ERROR_STREAM("Number of end point names and poses don't match! (" << request.endpt_names.size()
                     << ", " << request.endpt_poses.size() << ")");
    ROS_ERROR_STREAM("Aborting service call!");
    return false;
  }

  // extract current joint positions from the request
  KDL::JntArray q, q_desi;
  double nr_of_jnts = request.ik_seed_state.name.size();
  q.resize(nr_of_jnts);
  q_desi.resize(nr_of_jnts);
  for (unsigned int i=0; i < nr_of_jnts; ++i)
  {
    int tmp_index = getJointIndex(request.ik_seed_state.name[i]);
    if (tmp_index >=0)
    {
      q(tmp_index) = request.ik_seed_state.position[i];
      ROS_DEBUG_STREAM("Joint '" << request.ik_seed_state.name[i] << "' is now number '" << tmp_index << "'.");
    }
    else
    {
      ROS_ERROR_STREAM(i << ": No index for joint '" << request.ik_seed_state.name[i] << "'! Aborting service call.");
      return false;
    }
  }

  // convert pose messages into transforms from the root frame to the given poses and further into KDL::Frames
  geometry_msgs::PoseStamped pose_msg_in;
  geometry_msgs::PoseStamped pose_msg_transformed;
  KDL::Frames desired_poses;
  KDL::Frame desired_pose;
  for(unsigned int i = 0; i < request.endpt_names.size(); ++i)
  {
    pose_msg_in = request.endpt_poses[i];
    if (pose_msg_in.header.frame_id != tree_root_name_)
    {
      try
      {
        tf_listener_.waitForTransform(tree_root_name_, pose_msg_in.header.frame_id, pose_msg_in.header.stamp,
                                        ros::Duration(0.1));
        tf_listener_.transformPose(tree_root_name_, pose_msg_in, pose_msg_transformed);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_ERROR_STREAM("Could not transform pose from '" << pose_msg_in.header.frame_id << "' to frame '"
                         << tree_root_name_ <<  "!");
        ROS_DEBUG_STREAM(ex.what());
  //      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return false;
      }
      tf::poseMsgToKDL(pose_msg_transformed.pose, desired_pose);
    }
    else
    {
      tf::poseMsgToKDL(pose_msg_in.pose, desired_pose);
    }
    desired_poses[request.endpt_names[i]] = desired_pose;
  }

  // use the solver to compute desired joint positions
  ik_duration_ = ros::Time::now().toSec();
  int ik_ret = ik_pos_solver_->CartToJnt_it(q, desired_poses, q_desi); // NOTE: Before it was CartToJnt (without the _it). What's the difference?
  ik_duration_ = ros::Time::now().toSec() - ik_duration_;
  ik_duration_median_ = ((ik_duration_median_ * (loop_count_ - 1)) + ik_duration_) / loop_count_;

  // insert the solver's result into the service response
  if (ik_ret >= 0 || ik_ret == -3)
  {
    response.solution.header.stamp = ros::Time::now();
    response.solution.header.frame_id = tree_root_name_;
    response.solution.name = info_.joint_names;
    response.solution.position.resize(nr_of_jnts_);
    response.solution.velocity.resize(nr_of_jnts_);
    for (unsigned int i=0; i < nr_of_jnts_; ++i)
    {
      response.solution.position[i] = q_desi(i);
      response.solution.velocity[i] = (q_desi(i) - q(i)) * parameters_.ik_call_frequency;
      ROS_DEBUG_STREAM("IK Solution: " << response.solution.name[i] << " " << i << ": pos = "
                       << response.solution.position[i] << ", vel = " << response.solution.velocity[i]);
    }
//    response.error_code.val = response.error_code.SUCCESS;

    ik_srv_duration_ = ros::Time::now().toSec() - ik_srv_duration_;
    ik_srv_duration_median_ = ((ik_srv_duration_median_ * (loop_count_ - 1)) + ik_srv_duration_) / loop_count_;
    loop_count_++;
    ROS_INFO_STREAM_THROTTLE(1.0, "tree_kinematics: current/median IK duration: " << ik_duration_
                             << "s/" << ik_duration_median_ << "s.");
    ROS_INFO_STREAM_THROTTLE(1.0, "tree_kinematics: current/median IK service duration: " << ik_srv_duration_
                             << "s/" << ik_srv_duration_median_ << "s.");

    if (ik_ret == -3)
    {
      ROS_DEBUG_STREAM("Maximum iterations reached! (error code = " << ik_ret << ")");
    }
  }
  else
  {
    ROS_WARN_STREAM("An IK solution could not be found (error code = " << ik_ret << ")");
//    response.error_code.val = response.error_code.NO_IK_SOLUTION;

    ik_srv_duration_ = ros::Time::now().toSec() - ik_srv_duration_;
    ik_srv_duration_median_ = ((ik_srv_duration_median_ * (loop_count_ - 1)) + ik_srv_duration_) / loop_count_;
    loop_count_++;
    ROS_DEBUG_STREAM_THROTTLE(1.0, "tree_kinematics: current/median IK duration: " << ik_duration_
                             << "s/" << ik_duration_median_ << "s.");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "tree_kinematics: current/median IK service duration: " << ik_srv_duration_
                             << "s/" << ik_srv_duration_median_ << "s.");
  }
  return true;
}

} // namespace

