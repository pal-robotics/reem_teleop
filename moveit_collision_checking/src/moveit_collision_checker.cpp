/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/moveit_collision_checking/LICENSE
 **/
 
#include "moveit_collision_checking/moveit_collision_checker.h"


namespace motion_retargeting
{

MoveItCollisionChecker::MoveItCollisionChecker(const std::string& robot_description)
{
  tf_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(2.0)));
  planning_scene_monitor_ = planning_scene_monitor::PlanningSceneMonitorPtr(
                            new planning_scene_monitor::PlanningSceneMonitor(robot_description, tf_));

  if (planning_scene_monitor_->getPlanningScene())
  {
    ROS_DEBUG_STREAM("MoveIt Collision Checker: Starting context monitors...");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();
    ROS_DEBUG_STREAM("MoveIt Collision Checker: Context monitors started.");
  }
  else
  {
    ROS_ERROR_STREAM("MoveIt Collision Checker: Planning scene not configured!");
  }
};

bool MoveItCollisionChecker::checkState(const sensor_msgs::JointState& joint_states)
{
  collision_result_.clear();
  current_planning_scene_ = planning_scene_monitor_->getPlanningScene();
  robot_state::RobotState& current_state = current_planning_scene_->getCurrentStateNonConst();
//  ROS_DEBUG_STREAM("Current state: " << current_state);
  for (unsigned int joint = 0; joint < joint_states.name.size(); ++joint)
  {
    double* joint_position = new double(joint_states.position[joint]);
    current_state.setJointPositions(joint_states.name[joint], joint_position);
  }

  collision_request_.contacts = true;
  collision_request_.max_contacts = 1000;
  current_planning_scene_->checkSelfCollision(collision_request_, collision_result_);
  if (collision_result_.collision)
  {
    ROS_DEBUG_STREAM("MoveIt Collision Checker: Current state is in self-collision!");
    for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result_.contacts.begin();
        it != collision_result_.contacts.end(); ++it)
    {
      ROS_DEBUG_STREAM("Contact between: " << it->first.first << " and " << it->first.second);
    }
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM("MoveIt Collision Checker: Current state is not in self-collision.");
    return true;
  }
};

}; //namespace motion_retargeting
