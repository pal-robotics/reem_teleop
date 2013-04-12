/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
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

#include "motion_adaption/motion_adaption.h"
#include "motion_adaption/types/trans_rot_adaption.h"

namespace motion_adaption
{

MotionAdaption::MotionAdaption(const std::vector<MotionAdaptionParameters>& adaption_parameters)
{
  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);
  tf_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
  internal_tf_ = boost::shared_ptr<tf::Transformer>(new tf::Transformer);
  ROS_DEBUG_STREAM("Motion adaption: Will prepare " << adaption_parameters.size() << " adaption(s).");
  for (unsigned int param = 0; param < adaption_parameters.size(); ++param)
  {
    if (adaption_parameters[param].adaption_type == MotionAdaptionParameters::TransRotAdaption)
    {
      AdaptionTypePtr new_adaption(new TransRotAdaption(adaption_parameters[param],
                                                        tf_listener_,
                                                        tf_broadcaster_,
                                                        internal_tf_));
      adaptions_.push_back(new_adaption);
      ROS_DEBUG_STREAM("Motion adaption: Adaption '" << adaption_parameters[param].adaption_name << "("
                       << adaption_parameters[param].adaption_type << ") created.");
    }
  }
};

MotionAdaption::~MotionAdaption(){};

bool MotionAdaption::adapt(std::vector<geometry_msgs::PoseStamped>& adapted_poses_output)
{
  /*
   * Trigger adaption for each configured adaption type
   * TODO:
   * * Do we want synchronisation? E.g. first get _all_ transforms, then adapt _all_.
   */
  for (unsigned int adaption = 0; adaption < adaptions_.size(); ++adaption)
  {
    ROS_DEBUG_STREAM("Motion adaption: Triggering adaption type nr. " << adaption);
    if (adaptions_[adaption]->adapt(adapted_poses_))
    {
      for (unsigned int pose = 0; pose < adapted_poses_.size(); ++pose)
      {
        adapted_poses_output.push_back(adapted_poses_[pose]);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Motion adaption: Couldn't perform adaption '" << adaptions_[adaption]->getAdaptionName()
                       << "'. Aborting adaption.");
      return false;
    }
  }
  return true;
}

} // namespace motion_adaption

