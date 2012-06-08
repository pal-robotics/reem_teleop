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


#include <ros/ros.h>
#include "motion_adaption/motion_adaption.h"

/**
 * \brief Once started, this node periodically adapts the operator's transforms
 *
 * This node instantiates the motion_adaption class and
 * periodically adapts the currently available operator transforms.
 *
 * @param argc not used
 * @param argv not used
 * @return always 0
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_adaption_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30.0);
  unsigned int loop_count = 1;
  double cycle_time_median = 0.0;
  MotionAdaption motion_adaption;

  while (nh.ok())
  {
    motion_adaption.adapt();

    loop_rate.sleep();

    cycle_time_median = ((cycle_time_median * (loop_count - 1)) + loop_rate.cycleTime().toSec()) / loop_count;

    ROS_DEBUG_THROTTLE(1.0, "motion_adaption: cycle time %f and median cycle time %f", loop_rate.cycleTime().toSec(),
    cycle_time_median);

    loop_count++;
  }
  return 0;
}

