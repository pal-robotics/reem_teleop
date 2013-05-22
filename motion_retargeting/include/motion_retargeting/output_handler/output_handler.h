/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/output_handler.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 13, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef OUTPUT_HANDLER_H_
#define OUTPUT_HANDLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>

namespace motion_retargeting
{

/**
 * Base class for specific output handlers
 */
class OutputHandler
{
public:
  OutputHandler(){};
  virtual ~OutputHandler(){};
  /**
   * Every output handler needs to implement this function for publishing the goal joint states
   * @param output_joint_states goal joint states
   * @return true, if no errors occur
   */
  virtual bool setOutput(const sensor_msgs::JointState& output_joint_states) = 0;
private:
};

typedef boost::shared_ptr<OutputHandler> OutputHandlerPtr;

} // namespace motion_retargeting

#endif /* OUTPUT_HANDLER_H_ */
