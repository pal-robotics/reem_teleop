/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/openni_tracker_input_handler.h
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

#ifndef OPENNI_TRACKER_INPUT_HANDLER_H_
#define OPENNI_TRACKER_INPUT_HANDLER_H_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include "input_handler.h"

namespace motion_retargeting
{

class OpenNIInputHandler
{
public:
  OpenNIInputHandler(){};
  ~OpenNIInputHandler(){};
  virtual bool getInput(std::vector<tf::TransformStamped>& input_transforms)
  {
    return true;
  };
private:

};

} // namespace motion_retargeting

#endif /* OPENNI_TRACKER_INPUT_HANDLER_H_ */
