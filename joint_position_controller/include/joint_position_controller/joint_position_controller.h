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


#include <kdl/jntarray.hpp>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_mechanism_model/tree.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class JointPositionController: public pr2_controller_interface::Controller
{
  private:  
  // The current robot state (to get the time stamp)              
  pr2_mechanism_model::RobotState* robot_state_;
  
  // The tree handle (i.e. for getting current joint positions)
  pr2_mechanism_model::Tree tree_;
  
  unsigned int nrOfJnts_;           // Number of joints  
  ros::Time         last_time_;     // Variable used for calculating the timestep
  double            dt_;            // Timestep
  unsigned int      loop_count_;    // Used to control different cycletimes
  
  // KDL variables (which need to be pre-allocated). 
  KDL::JntArray     q_;             // Joint positions
  KDL::JntArray     q_desi_;        // Desired joint positions

  // Parameters for the joint controller
  KDL::JntArray     q_err_;         // Joints' position error
  KDL::JntArray     q_err_old_;     // Former joints' position error  
  KDL::JntArray     q_err_dot_;     // Change of joints' position error    
  bool              limit_error_;   // flag indicating whether or not to limit the error before calculating efforts
  
  double            q_err_max_;     // Maximum error taken into account
  double            q_err_dot_max_; // Maximum error velocity taken into account per timestep

  KDL::JntArray     Kp_;            // Proportional gains    
  KDL::JntArray     Kd_;            // Derivative gains
  KDL::JntArray     tau_;           // Joint torques
  KDL::JntArray     tau_max_;       // Maximum joint torques
  
  // Subscriber to joint states commands
  ros::Subscriber sub_joint_states_cmd;
  
  // Callback function for the subscriber for joint states commands
  void jointStatesCmdCB(const sensor_msgs::JointState::ConstPtr &command);
  
  public:
    bool init(pr2_mechanism_model::RobotState*, ros::NodeHandle&);
    void starting();
    void update();
    void stopping();
};

