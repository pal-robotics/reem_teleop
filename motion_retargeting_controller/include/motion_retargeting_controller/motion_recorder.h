/**
 * @file /motion_retargeting_controller/include/motion_retargeting_controller/motion_recorder.h
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 24, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_RECORDER_H_
#define MOTION_RECORDER_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>

namespace motion_retargeting
{

/**
 * Base class for specific motion recorders handlers
 */
class MotionRecorder
{
public:
  MotionRecorder(){};
  virtual ~MotionRecorder(){};
  /**
   * Prepares the recorder for storing motion
   * @return true, if no errors occur
   */
  virtual bool prepareRecording() = 0;
  /**
   * Stops recording and saves the recorded motion
   * @return true, if no errors occur
   */
  virtual bool stopRecording() = 0;
  /**
   * Stores a single part of the motion
   * @param joint_states joint states to be stored
   * @return true, if no errors occur
   */
  virtual bool storeMotion(const sensor_msgs::JointState& joint_states) = 0;
  /**
   * Prepares the recorder for playing back the stored motion
   * @return true, if no errors occur
   */
  virtual bool preparePlayback(std::string& filename) = 0;
  /**
   * Stops the playback
   * @return true, if no errors occur
   */
  virtual bool stopPlayback() = 0;
  /**
   * Reads one part of the stored motion
   * @param joint_states joint states read from the storage
   * @return true, if no errors occur
   */
  virtual bool readMotion(sensor_msgs::JointState& joint_states) = 0;
private:
};

typedef boost::shared_ptr<MotionRecorder> MotionRecorderPtr;

} // namespace motion_retargeting

#endif /* MOTION_RECORDER_H_ */
