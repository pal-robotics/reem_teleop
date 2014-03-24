/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_retargeting/LICENSE
 **/

#include "motion_retargeting_msgs/utils.h"

namespace motion_retargeting_msgs
{

std::string errorCodeToString(motion_retargeting_msgs::PlaybackStatusCodes code)
{
  std::string message("");

  if (code.value == motion_retargeting_msgs::PlaybackStatusCodes::GENERAL_ERROR)
  {
    message = "General error";
  }
  else if (code.value == motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK_ERROR)
  {
    message = "Playback error";
  }
  else if (code.value == motion_retargeting_msgs::PlaybackStatusCodes::IDLING)
  {
    message = "Idling";
  }
  else if (code.value == motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK)
  {
    message = "Playback";
  }
  else if (code.value == motion_retargeting_msgs::PlaybackStatusCodes::PLAYBACK_FINISHED)
  {
    message = "Playback finished";
  }
  else
  {
    std::cout << "Unknown error code: '" << code << "' !" << std::endl;
  }

  return message;
}

} //namespace
