/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/input_handler.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 16, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef INPUT_HANDLER_H_
#define INPUT_HANDLER_H_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <vector>
#include <boost/shared_ptr.hpp>
#include <tf/transform_datatypes.h>

namespace motion_retargeting
{

/**
 * Base class for specific input handlers
 */
class InputHandler
{
public:
  InputHandler(){};
  virtual ~InputHandler(){};
  /**
   * Every input handler needs to implement this function for retrieving the necessary transforms for motion adaption
   * @param output_joint_states goal joint states
   * @return true, if no errors occur
   */
  virtual bool getInput(std::vector<tf::TransformStamped>& input_transforms) = 0;
private:
};

typedef boost::shared_ptr<InputHandler> InputHandlerPtr;

} // namespace motion_retargeting

#endif /* INPUT_HANDLER_H_ */
