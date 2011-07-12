/*
 * Filename: reem_teleop.cpp
 * Author: Marcus Liebhardt
 * Created: 17.03.2011
 * License:
 * Description: if new adjusted operator's motion has arrived, this node calculates the desired joint position using 
 * KDL's tree inverse kinematics solvers, afterwards the result gets checked for self-collision.
 * If there is no self-collision the desired joint positions get sent to the reem_joint_controller.
 */


#include <string>
#include <map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tree_kinematics/get_tree_position_ik.h>


static const std::string FK_SERVICE = "/tree_kinematics_node/get_position_fk";
static const std::string IK_SERVICE = "/tree_kinematics_node/get_position_ik";
static const std::string CC_SERVICE = "/upper_body_environment_server/get_state_validity";
static const std::string PUB_TOPIC_JOINT_STATES_CMD = "/joint_states_cmd";
static const std::string SUB_TOPIC_JOINT_STATES = "/joint_states";

bool joint_states_valid = false;
sensor_msgs::JointState old_joint_state;

// pointer to the current joint states
sensor_msgs::JointState::ConstPtr joint_states_ptr;


void jointStatesCB(const sensor_msgs::JointState::ConstPtr &joint_states)
{  
  joint_states_ptr = joint_states;
  if(!joint_states_valid)
    old_joint_state = *joint_states;
  joint_states_valid = true;
}

bool getGoalTransform(tf::TransformListener& tf_listener,
                      std::string& root_frame_name,
                      std::string goal_frame_name,
                      geometry_msgs::PoseStamped& pose)
{
  tf::StampedTransform transform;
  try
  {
    tf_listener.waitForTransform(root_frame_name, goal_frame_name, ros::Time(0), ros::Duration(0.5));
    tf_listener.lookupTransform(root_frame_name, goal_frame_name, ros::Time(0), transform);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_DEBUG("%s",ex.what());
    ROS_WARN("No transformation available from '%s' to '%s'", root_frame_name.c_str(), goal_frame_name.c_str());
    return false;
  }
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = root_frame_name;
  pose.pose.position.x = transform.getOrigin().x();
  pose.pose.position.y = transform.getOrigin().y();
  pose.pose.position.z = transform.getOrigin().z();
  pose.pose.orientation.x = transform.getRotation().x();
  pose.pose.orientation.y = transform.getRotation().y();
  pose.pose.orientation.z = transform.getRotation().z();
  pose.pose.orientation.w = transform.getRotation().w();

  return true;  
}

  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "reem_teleop_coordinator_node");
  ros::NodeHandle nh, nh_private("~");

  int nr_of_endpts = 0;
  std::vector<std::string> endpts;
  endpts.clear();
  if(nh_private.getParam("nr_of_endpoints", nr_of_endpts) && nr_of_endpts >= 0)
  {
    for(int i = 0; i < nr_of_endpts; ++i)
    {
      std::string elementname;
      std::string endpt_name;    
      std::stringstream ss;
      ss << "endpoint_" << i;
      ss >> elementname;
      if(nh_private.getParam(elementname, endpt_name))  
      {
        endpts.push_back(endpt_name);
        ROS_INFO("Added endpoint '%s'", endpts[i].c_str());
      }
      else
      {
        ROS_FATAL("Couldn't get the name of an endpoint! Aborting...");
        return 0;
      }
    }
  }
  
  geometry_msgs::PoseStamped pose;
  std::map<std::string, geometry_msgs::PoseStamped> poses;
  std::map<std::string, std::string> rel_endpoints_goals; 
  for(unsigned int i = 0; i < endpts.size(); ++i)
  {
    std::string elementname;
    std::string endpt_goal_name;    
    std::stringstream ss;
    ss << "endpoint_" << i << "_goal";
    ss >> elementname;
    if(nh_private.getParam(elementname, endpt_goal_name))  
    {
      poses.insert(std::map<std::string, geometry_msgs::PoseStamped>::value_type(endpt_goal_name, pose));
      rel_endpoints_goals.insert(std::map<std::string, std::string>::value_type(endpt_goal_name, endpts[i])); 
      ROS_INFO("Added endpoint goal '%s'", endpt_goal_name.c_str());
      ROS_INFO("Goal '%s' is linked to endpoint '%s'", endpt_goal_name.c_str(), endpts[i].c_str());
    }
    else
    {
      ROS_FATAL("Couldn't get the name of an endpoint goal! Aborting...");
      return 0;
    }  
  }
  
  std::string root_frame_name;
  if(nh_private.getParam("root_frame_name", root_frame_name))  
    ROS_INFO("root frame name: %s", root_frame_name.c_str());
  else
  {
    ROS_FATAL("Couldn't get the name of the root frame! Aborting...");
    return 0;
  }
  
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transform;

  // subscriber for current joint states
  ros::Subscriber sub_joint_states = nh.subscribe(SUB_TOPIC_JOINT_STATES, 20, jointStatesCB); 
  
  // service client for IK calculations
  tree_kinematics::get_tree_position_ik tree_ik_srv;
  //ros::ServiceClient tree_ik_srv_client = nh.serviceClient<tree_kinematics::get_tree_position_ik>
  //(IK_SERVICE, true);

  // service client for FK calculations
  kinematics_msgs::GetPositionFK tree_fk_srv;
  //ros::ServiceClient tree_fk_srv_client = nh.serviceClient<kinematics_msgs::GetPositionFK>
  //(FK_SERVICE, true);
  
  // service client for self-collision and joint limits checking
  planning_environment_msgs::GetStateValidity::Request state_val_req;
  planning_environment_msgs::GetStateValidity::Response state_val_res;  
  //ros::ServiceClient check_state_validity_client = nh.serviceClient<planning_environment_msgs::GetStateValidity>
  //(CC_SERVICE, true);
  bool no_self_collision = false;
  bool check_self_collision, check_joint_limits = true;
  nh_private.param("check_self_collision", check_self_collision, true);
  nh_private.param("check_joint_limits", check_joint_limits, true);
  ROS_INFO("checking for self-collision: %s, checking for joint limits: %s", (check_self_collision)?"true":"false",
  (check_joint_limits)?"true":"false");
  
  // publisher for joint states commands
  ros::Publisher pub_joint_states_cmd = nh.advertise<sensor_msgs::JointState>(PUB_TOPIC_JOINT_STATES_CMD, 1);
  sensor_msgs::JointState joint_states_cmd;
  /*
  sensor_msgs::JointState old_joint_state;
  old_joint_state.name = std::vector<std::string>(24, "empty");
  old_joint_state.position = std::vector<double>(24, 0.0);
  old_joint_state.velocity = std::vector<double>(24, 0.0);
  old_joint_state.effort = std::vector<double>(24, 0.0);    
  */
  
  int loop_rate_value;
  nh_private.param("loop_rate", loop_rate_value, 10);
  ROS_INFO("loop rate: %d", loop_rate_value);
  ros::Rate loop_rate(loop_rate_value);
  unsigned int loop_count = 1;
  double cycle_time_median = 0.0;
  double ik_duration = 0.0;
  double ik_duration_median = 0.0;
  double scc_duration = 0.0;
  double scc_duration_median = 0.0;
  double cjp_duration = 0.0;
  double cjp_duration_median = 0.0;
  
  ros::service::waitForService(IK_SERVICE);
  ros::ServiceClient tree_ik_srv_client = nh.serviceClient<tree_kinematics::get_tree_position_ik>
  (IK_SERVICE, true);
  
  ros::service::waitForService(FK_SERVICE);
  ros::ServiceClient tree_fk_srv_client = nh.serviceClient<kinematics_msgs::GetPositionFK>
  (FK_SERVICE, true);
  
  ros::service::waitForService(CC_SERVICE);
  ros::ServiceClient check_state_validity_client = nh.serviceClient<planning_environment_msgs::GetStateValidity>
  (CC_SERVICE, true);
  
  while (nh.ok())
  {
    ros::spinOnce();
    
    // transform goal frames into pose messages      
    std::map<std::string, geometry_msgs::PoseStamped>::iterator poses_it;
    geometry_msgs::PoseStamped pose;
    bool goal_transforms_valid = true;
    for(poses_it = poses.begin(); poses_it != poses.end(); ++poses_it)
    {
      if(getGoalTransform(tf_listener, root_frame_name, poses_it->first, pose))
        poses_it->second = pose;
      else
      {
        ROS_WARN_THROTTLE(0.5, "No valid transformation available for endpoint goal '%s'", poses_it->first.c_str());
        ROS_WARN_THROTTLE(0.5, "No commands will be calculated.");
        goal_transforms_valid = false;
      }
    }
    
    if (joint_states_valid && goal_transforms_valid
    {
      // IK calculations

      // feed pose messages for every endpoint into the request
      tree_ik_srv.request.pos_ik_request.clear();
      kinematics_msgs::PositionIKRequest pos_ik_request;
      std::map<std::string, std::string>::iterator rel_it;
      for(poses_it = poses.begin(); poses_it != poses.end(); ++poses_it)
      {
        rel_it = rel_endpoints_goals.find(poses_it->first);
        pos_ik_request.ik_link_name = rel_it->second;
        pos_ik_request.pose_stamped = poses_it->second;
        tree_ik_srv.request.pos_ik_request.push_back(pos_ik_request);
      }

      // feeding current joint positions into the request
      if (joint_states_ptr)
        //tree_ik_srv.request.pos_ik_request[0].ik_seed_state.joint_state = *joint_states_ptr;
        tree_ik_srv.request.pos_ik_request[0].ik_seed_state.joint_state = old_joint_state;
      else
      {
        ROS_ERROR("joint_states_ptr invalid! Aborting loop ...");
        continue;
      }  
      /*
      ROS_DEBUG("Following information has been gathered:");
      for(unsigned int i = 0; i < tree_ik_srv.request.pos_ik_request.size(); ++i)
      {
        if( i == 0 )
        {
          for(unsigned int j = 0;
                           j < tree_ik_srv.request.pos_ik_request[i].ik_seed_state.joint_state.name.size();
                           ++j)
            ROS_DEBUG("joint[%d]: %s with position %f",
                      j,
                      tree_ik_srv.request.pos_ik_request[i].ik_seed_state.joint_state.name[j].c_str(),
                      tree_ik_srv.request.pos_ik_request[i].ik_seed_state.joint_state.position[j]);   
        }          
        ROS_DEBUG("link[%d]: %s with x = %f, y = %f, z = %f", i,
        tree_ik_srv.request.pos_ik_request[i].ik_link_name.c_str(),
        tree_ik_srv.request.pos_ik_request[i].pose_stamped.pose.position.x,
        tree_ik_srv.request.pos_ik_request[i].pose_stamped.pose.position.y,
        tree_ik_srv.request.pos_ik_request[i].pose_stamped.pose.position.z);
      }
      */
      
      ik_duration = ros::Time::now().toSec();
      if (tree_ik_srv_client.call(tree_ik_srv))
      {
        ROS_DEBUG_THROTTLE(1.0, "get_tree_position_ik service call was successful.");
        ROS_DEBUG_THROTTLE(1.0, "get_tree_position_ik service call response:");
        for(unsigned int i = 0; i < tree_ik_srv.response.solution.joint_state.name.size(); ++i)
        {
          ROS_DEBUG_THROTTLE(1.0, "desired position for joint[%d]('%s'): %f", i,
          tree_ik_srv.response.solution.joint_state.name[i].c_str(), 
          tree_ik_srv.response.solution.joint_state.position[i]);
        }
      }
      else
      {
        ROS_ERROR("get_tree_position_ik service call failed! Aborting loop ...");
        continue;
      }
      ik_duration = ros::Time::now().toSec() - ik_duration;
      ik_duration_median = ((ik_duration_median * (loop_count - 1)) + ik_duration) / loop_count;
      
      // self-collision checking
      state_val_req.robot_state = tree_ik_srv.response.solution;
      state_val_req.check_collisions = check_self_collision;
      state_val_req.check_joint_limits = check_joint_limits;
      
      scc_duration = ros::Time::now().toSec();
      if(check_state_validity_client.call(state_val_req, state_val_res))
      {
        if(state_val_res.error_code.val == state_val_res.error_code.SUCCESS)
        {
          ROS_DEBUG_THROTTLE(1.0, "Requested state is not in collision");
          no_self_collision = true;
        }
        else
          ROS_WARN_THROTTLE(0.5, "Requested state is in collision. Error code: %d", state_val_res.error_code.val);
      }
      else
      {
        ROS_ERROR("Service call to check state validity failed ('%s')! Aborting loop ...", 
        check_state_validity_client.getService().c_str());
        continue;
      }
      scc_duration = ros::Time::now().toSec() - scc_duration;
      scc_duration_median = ((scc_duration_median * (loop_count - 1)) + scc_duration) / loop_count;
      
      // commanding joint positions
      //no_self_collision = true;
      if(no_self_collision == true)
      {
        joint_states_cmd = tree_ik_srv.response.solution.joint_state;
        joint_states_cmd.header.stamp = ros::Time::now();
        pub_joint_states_cmd.publish(joint_states_cmd);
        old_joint_state = joint_states_cmd;
        no_self_collision = false;       
      }
      else
      {
        for(unsigned int i = 0; i < old_joint_state.name.size(); ++i)
          old_joint_state.velocity[i] = 0.0;
        joint_states_cmd = old_joint_state;
        joint_states_cmd.header.stamp = ros::Time::now();
        pub_joint_states_cmd.publish(joint_states_cmd);
      }
      

      // For debug purpose: publish forward kinematics messages on tf
      cjp_duration = ros::Time::now().toSec();
       
      tree_fk_srv.request.header.stamp = ros::Time::now();
      tree_fk_srv.request.header.frame_id = "/base_footprint";      
      tree_fk_srv.request.robot_state.joint_state = tree_ik_srv.response.solution.joint_state;
      tree_fk_srv.request.robot_state.joint_state = *joint_states_ptr;                  
      tree_fk_srv.request.fk_link_names.resize(8);
      tree_fk_srv.request.fk_link_names[0] = "arm_right_2_link";
      tree_fk_srv.request.fk_link_names[1] = "arm_right_4_link";
      tree_fk_srv.request.fk_link_names[2] = "palm_right_link";
      tree_fk_srv.request.fk_link_names[3] = "arm_left_2_link";      
      tree_fk_srv.request.fk_link_names[4] = "arm_left_4_link";      
      tree_fk_srv.request.fk_link_names[5] = "palm_left_link";
      tree_fk_srv.request.fk_link_names[6] = "head_2_link";
      tree_fk_srv.request.fk_link_names[7] = "torso_2_link";      
      if (tree_fk_srv_client.call(tree_fk_srv))
      {
        if(tree_fk_srv.request.fk_link_names.size() == tree_fk_srv.response.pose_stamped.size())
        {
          for(unsigned int i = 0; i < tree_fk_srv.response.pose_stamped.size(); ++i)
          {
            std::stringstream ss;
            std::string name;
            ss << "fk_" << tree_fk_srv.request.fk_link_names[i];
            ss >> name;
            transform.header.seq++;
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = tree_fk_srv.request.header.frame_id;
            transform.child_frame_id = name;   
            transform.transform.translation.x = tree_fk_srv.response.pose_stamped[i].pose.position.x;
            transform.transform.translation.y = tree_fk_srv.response.pose_stamped[i].pose.position.y;
            transform.transform.translation.z = tree_fk_srv.response.pose_stamped[i].pose.position.z;                    
            transform.transform.rotation = tree_fk_srv.response.pose_stamped[i].pose.orientation;          
            tf_broadcaster.sendTransform(transform);
          }
        }
      }
      else
      {
        ROS_ERROR("get_tree_position_fk service call failed! Aborting loop ...");
        continue;
      }
        
      cjp_duration = ros::Time::now().toSec() - cjp_duration;
      cjp_duration_median = ((cjp_duration_median * (loop_count - 1)) + cjp_duration) / loop_count; 
    }
    
    ros::spinOnce();

    loop_rate.sleep();
    
    cycle_time_median = ((cycle_time_median * (loop_count - 1)) + loop_rate.cycleTime().toSec()) / loop_count;

    ROS_INFO_THROTTLE(1.0, "reem_teleop: cycle time %f and median cycle time %f",
    loop_rate.cycleTime().toSec(), cycle_time_median);
    ROS_INFO_THROTTLE(1.0, "reem_teleop: IKC current duration %f and median %f", ik_duration, ik_duration_median);
    ROS_INFO_THROTTLE(1.0, "reem_teleop: SCC current duration %f and median %f", scc_duration, scc_duration_median);
    ROS_INFO_THROTTLE(1.0, "reem_teleop: FKC current duration %f and median %f", cjp_duration, cjp_duration_median);
    
    loop_count ++;
  }
  tree_ik_srv_client.shutdown();
  check_state_validity_client.shutdown();  
  pub_joint_states_cmd.shutdown();
  tree_fk_srv_client.shutdown();
  
  ROS_INFO("exiting ...");
  
  return 0;
}

