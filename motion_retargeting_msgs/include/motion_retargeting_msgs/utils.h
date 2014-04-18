/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_retargeting/LICENSE
 **/

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <motion_retargeting_msgs/PlaybackStatusCodes.h>

namespace motion_retargeting_msgs
{

/**
 * Returns a human-readable message for the given error code
 * If an invalid error code is provide, i.e. not in motion_retargeting_msgs/PlaybackStatusCodes,
 * then the return string is empty.
 *
 * @param code error code from motion_retargeting_msgs/PlaybackStatusCodes
 *
 * @return human-readable error message
 */
std::string errorCodeToString(motion_retargeting_msgs::PlaybackStatusCodes code);

} // namespace

#endif /* UTILS_H_ */
