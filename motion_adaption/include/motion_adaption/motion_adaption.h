/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
 *  Copyright (c) 2013, Yujin Robot
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Marcus Liebhardt */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MOTION_ADAPTION_H_
#define MOTION_ADAPTION_H_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "motion_adaption/types/adaption_type.h"

namespace motion_adaption
{

/**
 * \brief Adapts the tracked end points of the operator to the robot's body proportions
 *
 * The adaptions uses the transforms of the specified operator end points, scales them according to the robot's
 * body proportions and maps them onto the robot. The orientation of each end point is mapped unchanged.
 *
 */
class MotionAdaption
{
public:
  MotionAdaption(const std::vector<AdaptionParameters>& adaption_parameters);
  ~MotionAdaption();

  /**
   * This method consists of a hierarchy of all private methods of this class. Each method gets called,
   * if the previous method has finished successfully.
   */
  bool adapt(std::vector<geometry_msgs::PoseStamped>& adapted_poses_output);

  boost::shared_ptr<tf::TransformListener> tf_listener_;
  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  boost::shared_ptr<tf::Transformer> internal_tf_;
private:
  std::vector<AdaptionTypePtr> adaptions_;
  std::vector<geometry_msgs::PoseStamped> adapted_poses_;
};

typedef boost::shared_ptr<motion_adaption::MotionAdaption> MotionAdaptionPtr;

} // namespace motion_adaption

#endif /* MOTION_ADAPTION_H_ */
