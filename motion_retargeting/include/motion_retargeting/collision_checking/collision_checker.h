/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_retargeting/LICENSE
 **/

#ifndef MOTION_RETARGETING_COLLISION_CHECKER_H_
#define MOTION_RETARGETING_COLLISION_CHECKER_H_

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>

namespace motion_retargeting
{

/**
 * Base class for specific collision checking implementations
 */
class CollisionChecker
{
public:
  CollisionChecker(){};
  virtual ~CollisionChecker(){};
  /**
   * Every collision checker needs to implement this function for checking the current robot state for collisions
   * @param joint_states joint states with which collision checking should be done
   * @return true, if no collisions have been detected, false otherwise
   */
  virtual bool checkState(const sensor_msgs::JointState& joint_states) = 0;
private:
};

typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

} // namespace motion_retargeting

#endif /* MOTION_RETARGETING_COLLISION_CHECKER_H_ */
