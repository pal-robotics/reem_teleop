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


#include <urdf/joint.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>
#include "tree_kinematics/tree_kinematics.h"

static const std::string FK_SERVICE = "get_position_fk";
static const std::string IK_SERVICE = "get_position_ik";


namespace tree_kinematics
{

bool TreeKinematics::init()
{
  // get URDF XML
  std::string urdf_xml, full_urdf_xml;
  nh_.param("urdf_xml_model",urdf_xml,std::string("robot_description"));
  nh_.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server ...");
  std::string result;
  if (!nh_.getParam(full_urdf_xml, result))
  {
    ROS_FATAL("Could not load the xml from parameter server: %s!", urdf_xml.c_str());
    return false;
  }

  // data for the IK position solver obtained from the robot model
  KDL::JntArray     joint_min;             // Minimum joint positions
  KDL::JntArray     joint_max;             // Maximum joint positions
  KDL::JntArray     joint_vel_max;         // Maximum joint velocities
  // create robot model, KDL::Tree and load model parameters from the model
  if (!loadModel(result, kdl_tree_, tree_root_name_, nr_of_jnts_, joint_min, joint_max, joint_vel_max))
  {
    ROS_FATAL("Could not load model!");
    return false;
  }

  // data for the IK solvers obtained from the parameter server
  int nr_of_endpts = 0;
  std::vector<std::string> endpts;
  endpts.clear();
  if(nh_private_.getParam("nr_of_endpoints", nr_of_endpts) && nr_of_endpts >= 0)
  {
    for(int i = 0; i < nr_of_endpts; ++i)
    {
      std::string elementname;
      std::string endpt_name;
      std::stringstream ss;
      ss << "endpoint_" << i;
      ss >> elementname;
      if(nh_private_.getParam(elementname, endpt_name))
      {
        endpts.push_back(endpt_name);
        ROS_DEBUG("Added endpoint '%s'", endpts[i].c_str());
      }
      else
      {
        ROS_FATAL("Couldn't get the name of an endpoint!");
        return false;
      }
    }
  }
  nh_private_.param("srv_call_frequency", srv_call_frequency_, 10);
  double x_dot_trans_max, x_dot_rot_max, x_dot_trans_min, x_dot_rot_min;
  double q_dot_min_factor, q_dot_max_factor;
  double epsilon, low_pass_factor;
  int max_iterations;
  nh_private_.param("epsilon", epsilon, 1e-3);
  nh_private_.param("max_iterations", max_iterations, 10);
  nh_private_.param("low_pass_factor", low_pass_factor, 0.0);
  nh_private_.param("x_dot_trans_max", x_dot_trans_max, 1.0);
  nh_private_.param("x_dot_rot_max", x_dot_rot_max, 1.0);
  nh_private_.param("x_dot_trans_min", x_dot_trans_min, 0.0);
  nh_private_.param("x_dot_rot_min", x_dot_rot_min, 0.0);
  nh_private_.param("q_dot_max_factor", q_dot_max_factor, 1.0);
  nh_private_.param("q_dot_min_factor", q_dot_min_factor, 0.0);
  x_dot_trans_max = x_dot_trans_max / srv_call_frequency_;
  x_dot_rot_max = x_dot_rot_max / srv_call_frequency_;
  x_dot_trans_min = x_dot_trans_min / srv_call_frequency_;
  x_dot_rot_min = x_dot_rot_min / srv_call_frequency_;
  q_dot_max_factor = q_dot_max_factor / srv_call_frequency_;
  q_dot_min_factor = q_dot_min_factor / srv_call_frequency_;
  KDL::Multiply(joint_vel_max,  q_dot_max_factor, joint_vel_max);
  KDL::JntArray joint_vel_min(nr_of_jnts_); // Minimum joint velocities
  for (unsigned int i = 0; i < joint_vel_min.rows(); ++i)
  {
    joint_vel_min(i) = 1.0;
  }
  KDL::Multiply(joint_vel_min,  q_dot_min_factor, joint_vel_min);

  ROS_DEBUG("srv_call_frequency = %d", srv_call_frequency_);
  ROS_DEBUG("x_dot_trans_max = %f, x_dot_rot_max = %f", x_dot_trans_max, x_dot_rot_max);
  ROS_DEBUG("x_dot_trans_min = %f, x_dot_rot_min = %f", x_dot_trans_min, x_dot_rot_min);
  ROS_DEBUG("q_dot_max_factor = %f", q_dot_max_factor);
  ROS_DEBUG("q_dot_min_factor  = %f", q_dot_min_factor);
  ROS_DEBUG("maximum iterations = %d", max_iterations);
  ROS_DEBUG("precision = %f", epsilon);
  ROS_DEBUG("low_pass_factor  = %f", low_pass_factor);

  // Configure kdl's kinematic solvers
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(kdl_tree_));
  ik_vel_solver_.reset(new KDL::TreeIkSolverVel_wdls(kdl_tree_, endpts));
  ik_pos_solver_.reset(new KDL::TreeIkSolverPos_Online(nr_of_jnts_,
                                                       endpts,
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
                                                       low_pass_factor,
                                                       max_iterations,
                                                       epsilon));

  // Configuration of the IK velocity solver
  // Joint space weight matrix
  js_w_matr_ = KDL::MatrixXd::Identity(nr_of_jnts_, nr_of_jnts_);
  ROS_DEBUG("filling the joint space weight matrix:");
  for (unsigned int i = 0 ; i < nr_of_jnts_ ; ++i)
  {
    std::string elementname;
    std::stringstream ss;
    ss << "js_w_matr/j" << i;
    ss >> elementname;
    double elementvalue;
    nh_private_.param(elementname, elementvalue, 1.0);
    js_w_matr_( i, i ) = elementvalue;
    ROS_DEBUG("qs_w_matr_(%d,%d) = %f", i, i, js_w_matr_( i, i ));
  }
  ik_vel_solver_->setWeightJS(js_w_matr_);

  // Task space weight matrix
  ts_w_matr_ = KDL::MatrixXd::Identity( 6 * endpts.size(), 6 * endpts.size() );
  std::vector<std::string> ending;
  ending.clear();
  ending.push_back("px");
  ending.push_back("py");
  ending.push_back("pz");
  ending.push_back("rx");
  ending.push_back("ry");
  ending.push_back("rz");
  ROS_DEBUG("filling the task space weight matrix:");
  for (unsigned int i = 0 ; i < endpts.size() ; ++i)
  {
    for (unsigned int j = 0 ; j < 6; ++j)
    {
      std::string elementname;
      std::stringstream ss;
      ss << "ts_w_matr/endpt" << i << "/";
      ss << ending[j];
      ss >> elementname;
      double elementvalue;
      nh_private_.param(elementname, elementvalue, 0.0);
      ts_w_matr_( i * 6 + j, i * 6 + j ) = elementvalue;
      ROS_DEBUG("ts_w_matr_(%d,%d) = %f", i * 6 + j, i * 6 + j, ts_w_matr_( i * 6 + j, i * 6 + j ));
    }
  }
  ik_vel_solver_->setWeightTS(ts_w_matr_);

  // lambda
  nh_private_.param("lambda", lambda_, 0.01);
  ROS_DEBUG("lambda = %f", lambda_);
  ik_vel_solver_->setLambda(lambda_);

  fk_service_ = nh_private_.advertiseService(FK_SERVICE, &TreeKinematics::getPositionFk, this);
  ik_service_ = nh_private_.advertiseService(IK_SERVICE, &TreeKinematics::getPositionIk, this);

  return true;
}


bool TreeKinematics::loadModel(const std::string xml,
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


bool TreeKinematics::getPositionFk(kinematics_msgs::GetPositionFK::Request& request,
                                   kinematics_msgs::GetPositionFK::Response& response)
{
  ROS_DEBUG("getPositionFK method invoked.");
  KDL::JntArray q_in;
  double nr_of_jnts = request.robot_state.joint_state.name.size();
  q_in.resize(nr_of_jnts);

  for (unsigned int i=0; i < nr_of_jnts; ++i)
  {
    int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
    if (tmp_index >=0)
    {
      q_in(tmp_index) = request.robot_state.joint_state.position[i];
      ROS_DEBUG("joint '%s' is now number '%d'", request.robot_state.joint_state.name[i].c_str(), tmp_index);
    }
    else
    {
      ROS_FATAL("i: %d, No joint index for %s!", i, request.robot_state.joint_state.name[i].c_str());
      ROS_FATAL("Service call aborted.");
      return false;
    }
  }

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());
  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;
  for (unsigned int i=0; i < request.fk_link_names.size(); i++)
  {
    ROS_DEBUG("End effector name: %s",request.fk_link_names[i].c_str());
    int fk_ret = fk_solver_->JntToCart(q_in, p_out, request.fk_link_names[i]);
    if (fk_ret >= 0)
    {
      tf_pose.frame_id_ = tree_root_name_;
      tf_pose.stamp_ = ros::Time::now();
      tf::PoseKDLToTF(p_out, tf_pose);
      try
      {
        tf_listener_.transformPose(request.header.frame_id, tf_pose, tf_pose);
      }
      catch (tf::TransformException const &ex)
      {
        ROS_FATAL("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return false;
      }
      tf::poseStampedTFToMsg(tf_pose, pose);
      response.pose_stamped[i] = pose;
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    }
    else
    {
      ROS_WARN("A FK solution could not be found for %s (error code = %d)",
      request.fk_link_names[i].c_str(), fk_ret);
      response.error_code.val = response.error_code.NO_FK_SOLUTION;
    }
  }

  return true;
}


bool TreeKinematics::getPositionIk(tree_kinematics::get_tree_position_ik::Request &request,
                                   tree_kinematics::get_tree_position_ik::Response &response)
{
  ik_srv_duration_ = ros::Time::now().toSec();
  ROS_DEBUG("getPositionIK method invoked.");

  // extract current joint positions from the request
  KDL::JntArray q, q_desi;
  double nr_of_jnts = request.pos_ik_request[0].ik_seed_state.joint_state.name.size();
  q.resize(nr_of_jnts);
  q_desi.resize(nr_of_jnts);
  for (unsigned int i=0; i < nr_of_jnts; ++i)
  {
    int tmp_index = getJointIndex(request.pos_ik_request[0].ik_seed_state.joint_state.name[i]);
    if (tmp_index >=0)
    {
      q(tmp_index) = request.pos_ik_request[0].ik_seed_state.joint_state.position[i];
      ROS_DEBUG("joint '%s' is now number '%d'", request.pos_ik_request[0].ik_seed_state.joint_state.name[i].c_str(),
                                                tmp_index);
    }
    else
    {
      ROS_FATAL("i: %d, No joint index for %s!",i,request.pos_ik_request[0].ik_seed_state.joint_state.name[i].c_str());
      ROS_FATAL("Service call aborted.");
      return false;
    }
  }

  // convert pose messages into transforms from the root frame to the given poses and further into KDL::Frames
  geometry_msgs::PoseStamped pose_msg_in;
  geometry_msgs::PoseStamped pose_msg_transformed;
  tf::Stamped<tf::Pose> transform;
  tf::Stamped<tf::Pose> transform_root;
  KDL::Frames desired_poses;
  KDL::Frame desired_pose;
  for(unsigned int i = 0; i < request.pos_ik_request.size(); ++i)
  {
    pose_msg_in = request.pos_ik_request[i].pose_stamped;
    try
    {
      tf_listener_.waitForTransform(tree_root_name_, pose_msg_in.header.frame_id, pose_msg_in.header.stamp,
      ros::Duration(0.1));
      tf_listener_.transformPose(tree_root_name_, pose_msg_in, pose_msg_transformed);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_FATAL("%s",ex.what());
      ROS_FATAL("Could not transform IK pose from '%s' to frame '%s'", tree_root_name_.c_str(),
      pose_msg_in.header.frame_id.c_str());
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return false;
    }
    tf::PoseMsgToKDL(pose_msg_transformed.pose, desired_pose);
    desired_poses[request.pos_ik_request[i].ik_link_name] = desired_pose;
  }

  // use the solver to compute desired joint positions
  ik_duration_ = ros::Time::now().toSec();
  int ik_ret = ik_pos_solver_->CartToJnt(q, desired_poses, q_desi);
  ik_duration_ = ros::Time::now().toSec() - ik_duration_;
  ik_duration_median_ = ((ik_duration_median_ * (loop_count_ - 1)) + ik_duration_) / loop_count_;

  // insert the solver's result into the service response
  if (ik_ret >= 0 || ik_ret == -3)
  {
    response.solution.joint_state.name = info_.joint_names;
    response.solution.joint_state.position.resize(nr_of_jnts_);
    response.solution.joint_state.velocity.resize(nr_of_jnts_);
    for (unsigned int i=0; i < nr_of_jnts_; ++i)
    {
      response.solution.joint_state.position[i] = q_desi(i);
      response.solution.joint_state.velocity[i] = (q_desi(i) - q(i)) * srv_call_frequency_;
      ROS_DEBUG("IK Solution: %s %d: pos = %f, vel = %f",response.solution.joint_state.name[i].c_str(), i,
      response.solution.joint_state.position[i], response.solution.joint_state.velocity[i]);
    }
    response.error_code.val = response.error_code.SUCCESS;

    ik_srv_duration_ = ros::Time::now().toSec() - ik_srv_duration_;
    ik_srv_duration_median_ = ((ik_srv_duration_median_ * (loop_count_ - 1)) + ik_srv_duration_) / loop_count_;
    loop_count_++;

    ROS_INFO_THROTTLE(1.0, "tree_kinematics: ik_srv_duration %f and median %f", ik_srv_duration_,
    ik_srv_duration_median_);
    ROS_INFO_THROTTLE(1.0, "tree_kinematics: ik_duration %f and median %f", ik_duration_, ik_duration_median_);

    if (ik_ret == -3)
    {
      ROS_WARN_THROTTLE(1.0, "Maximum iterations reached! (error code = %d)", ik_ret);
    }
  }
  else
  {
    ROS_WARN("An IK solution could not be found (error code = %d)", ik_ret);
    response.error_code.val = response.error_code.NO_IK_SOLUTION;

    ik_srv_duration_ = ros::Time::now().toSec() - ik_srv_duration_;
    ik_srv_duration_median_ = ((ik_srv_duration_median_ * (loop_count_ - 1)) + ik_srv_duration_) / loop_count_;
    loop_count_++;

    ROS_INFO_THROTTLE(1.0, "tree_kinematics: ik_srv_duration %f and median %f", ik_srv_duration_,
    ik_srv_duration_median_);
    ROS_INFO_THROTTLE(1.0, "tree_kinematics: ik_duration %f and median %f", ik_duration_, ik_duration_median_);
  }
  return true;
}

} // namespace

