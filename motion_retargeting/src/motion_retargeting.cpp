/**
 * License: BSD
 *   https://raw.github.com/pal-robotics/reem_teleop/hydro-devel/motion_retargeting/LICENSE
 **/

#include <stdexcept>
#include "motion_retargeting/motion_retargeting.h"

namespace motion_retargeting
{

MotionRetargeting::MotionRetargeting(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private,
                                     const motion_adaption::MotionAdaptionPtr motion_adaption,
                                     const tree_kinematics::KinematicsParameters& kinematics_params,
                                     const tree_kinematics::TreeKinematicsPtr tree_kinematics,
                                     const CollisionCheckerPtr collision_checker,
                                     const OutputHandlerPtr output_handler,
                                     const MotionRecorderPtr motion_recorder) :
                                     nh_(nh),
                                     nh_private_(nh_private),
                                     joint_states_initialised_(false),
                                     check_collisions_(true),
                                     process_output_(true),
                                     record_motion_(false)
{
  if (motion_adaption)
  {
    motion_adaption_ = motion_adaption;
  }
  else
  {
    throw std::invalid_argument("No valid pointer to motion adaption provided!");
  }
  if (tree_kinematics)
  {
    tree_kinematics_ = tree_kinematics;
    joint_states_subscriber_ = nh_.subscribe("joint_states", 10, &MotionRetargeting::jointStatesCallback, this);
    tree_ik_request_.endpt_names = kinematics_params.endpt_names;
  }
  else
  {
    throw std::invalid_argument("No valid pointer to tree kinematics provided!");
  }
  if (output_handler)
  {
    output_handler_ = output_handler;
    output_control_subscriber_ = nh_private_.subscribe("output_control", 10,
                                                       &MotionRetargeting::outputControlCallback, this);
  }
  else
  {
    throw std::invalid_argument("No valid pointer to an output handler provided!");
  }
  if (collision_checker)
  {
    collision_checker_ = collision_checker;
    collision_checker_subscriber_ = nh_private_.subscribe("check_collisions", 10,
                                                          &MotionRetargeting::collisionCheckerCallback, this);
  }
  else
  {
    ROS_WARN_STREAM("Motion retargeting: No collision checker provided. Motions will not be checked for collisions!");
  }
  if (motion_recorder)
  {
    motion_recorder_ = motion_recorder;
    motion_recorder_subscriber_ = nh_private_.subscribe("record_motion", 10,
                                                        &MotionRetargeting::motionRecorderCallback, this);
  }
  else
  {
    ROS_WARN_STREAM("Motion retargeting: No motion recorder provided. Motion recording disabled.");
  }
}

MotionRetargeting::~MotionRetargeting(){};

void MotionRetargeting::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  if(!joint_states_initialised_)
  {
    joint_states_ = *joint_states;
    joint_states_initialised_ = true;
    ROS_DEBUG_STREAM("Joint states initialised.");
  }
  joint_states_ = *joint_states;
  return;
}

void MotionRetargeting::collisionCheckerCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if(check_collisions_)
  {
    check_collisions_ = false;
    ROS_WARN_STREAM("Motion Retargeting Controller: Collision checking disabled!");
  }
  else
  {
    check_collisions_ = true;
    ROS_INFO_STREAM("Motion Retargeting Controller: Collision checking enabled.");
  }
  return;
}

void MotionRetargeting::motionRecorderCallback(const std_msgs::Empty::ConstPtr& msg)
{
  // This is not multi-threading safe! Do not use this callback manually!
  if(record_motion_)
  {
    if (!motion_recorder_->stopRecording())
    {
      ROS_ERROR_STREAM("Motion Retargeting: Error occured while stoping motion recording.");
    }
    record_motion_ = false;
    ROS_INFO_STREAM("Motion Retargeting Controller: Motion recording deactivated.");
  }
  else
  {
    /*
     * TODO: Spin off a separate thread for the recorder in order to avoid slowing down retargeting
     *       (exchange date via shared queues)
     */
    try
    {
      if (motion_recorder_->prepareRecording())
      {
        record_motion_ = true;
        ROS_INFO_STREAM("Motion Retargeting Controller: Motion recording activated.");
      }
      else
      {
        ROS_ERROR_STREAM("Motion Retargeting: Error occured while preparing for motion recording.");
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Motion retargeting: Couldn't initialise the motion recorder!");
      ROS_DEBUG_STREAM(e.what());
    }
  }
  return;
}

void MotionRetargeting::outputControlCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if(process_output_)
  {
    process_output_ = false;
    ROS_INFO_STREAM("Motion Retargeting Controller: Output processing has been deactivated.");
  }
  else
  {
    process_output_ = true;
    ROS_INFO_STREAM("Motion Retargeting Controller: Output processing has been activated.");
  }
  return;
}

bool MotionRetargeting::retarget()
{
  if (motion_adaption_ && tree_kinematics_ && output_handler_)
  {
    adapted_entpt_poses_.clear();
    /*
     * Motion adaption
     */
    if(!(motion_adaption_->adapt(adapted_entpt_poses_)))
    {
      ROS_WARN_STREAM("Motion Retargeting Controller: Motion adaption failed. Skipping.");
      return false;
    }

    /*
     * Tree kinematics
     */
    if(joint_states_initialised_)
    {
      tree_ik_request_.endpt_poses = adapted_entpt_poses_;
      tree_ik_request_.ik_seed_state = joint_states_;
      if (tree_kinematics_->getPositionIk(tree_ik_request_, tree_ik_response_))
      {
        goal_joint_states_ = tree_ik_response_.solution;
      }
      else
      {
        ROS_WARN_STREAM("Motion Retargeting Controller: IK failed. Skipping.");
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM("Motion Retargeting Controller: Can't perform IK,"
                      << " since no joint states have been retrieved yet.");
      return false;
    }

    /*
     * Collision checking
     */
    if (collision_checker_ && check_collisions_)
    {
      if (!(collision_checker_->checkState(goal_joint_states_)))
      {
        ROS_WARN_STREAM("Motion Retargeting Controller: Computed motion is not collision-free. Skipping.");
        return false;
      }
    }

    /*
     * Recording
     */
    if (motion_recorder_ && record_motion_)
    {
      if(!(motion_recorder_->storeMotion(goal_joint_states_)))
      {
        ROS_WARN_STREAM("Motion Retargeting Controller: Storing the motion failed!");
        return false;
      }
    }

    /*
     * Publish the results
     */
    if (output_handler_ && process_output_)
    {
      if(!(output_handler_->setOutput(goal_joint_states_)))
      {
        ROS_WARN_STREAM("Motion Retargeting Controller: Publishing the goal joint states failed!");
        return false;
      }
    }
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Motion Retargeting: Can't retarget, since motion retargeting hasn't been set up correctly!");
    return false;
  }
}

} // namespace motion_retargeting
