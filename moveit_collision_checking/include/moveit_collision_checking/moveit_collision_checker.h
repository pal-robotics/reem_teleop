/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/moveit_collision_checking/LICENSE
 **/

#ifndef MOTION_RETARGETING_MOVEIT_COLLISION_CHECKER_H_
#define MOTION_RETARGETING_MOVEIT_COLLISION_CHECKER_H_

#include <string>
#include <ros/ros.h>
#include <motion_retargeting/collision_checking/collision_checker.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

namespace motion_retargeting
{

/**
 * Collision checking using MoveIt!
 */
class MoveItCollisionChecker : public CollisionChecker
{
public:
  /**
   * Constructs and starts the planning scene monitor
   *
   * @param robot_description name of the parameter containing the robot model in URDF
   *
   * TODO: Throw an exception, if sth fails
   */
  MoveItCollisionChecker(const std::string& robot_description = std::string("robot_description"));

  ~MoveItCollisionChecker(){};

  /**
   * Triggers the collision check
   *
   * TODO: Currently only does checks for self-collision.
   *       Investigate, if full collision checking is desired and possible
   *
   * @param joint_states joint states of the robot state to be checked for collisions
   * @return true, if no collision has been detected, false otherwise
   */
  bool checkState(const sensor_msgs::JointState& joint_states);

private:
  /**
   * Listens to the current transforms
   */
  boost::shared_ptr<tf::TransformListener> tf_;
  /**
   * Monitors the current planning scene
   */
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  /**
   * Pointer to the current planning scene
   */
  planning_scene::PlanningScenePtr current_planning_scene_;
  /**
   * Data required for a collision check request
   */
  collision_detection::CollisionRequest collision_request_;
  /**
   * Result of the collision check
   */
  collision_detection::CollisionResult collision_result_;
};

} // namespace motion_retargeting

#endif /* MOTION_RETARGETING_MOVEIT_COLLISION_CHECKER_H_ */
