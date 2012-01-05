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

//#include <vector>

//#include <Eigen/Core>
//#include <bullet/btQuaternion.h>

#include <LinearMath/btVector3.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


class MotionAdaption
{
  public:
    MotionAdaption();
    ~MotionAdaption();

    /// 
    void adapt();

  private:
    //internal tf library
    tf::Transformer internal_tf;
    ros::Time calc_time;
    ///
    bool getTransforms();

    ///
    bool setRefFrame();

    ///
    bool adaptTransforms();

    ///
    bool adaptTorso();
    
    ///
    bool adaptHead();
    
    ///
    bool scaleUserHandsAndElbows();
    
    ///
    bool adaptShoulders();
    
    ///
    bool adaptElbows();

    ///
    bool adaptHands();    
    
    ///
    void setGoals();

    ///
    void publishData();

    ros::NodeHandle nh_, nh_private_;

    /// listens to the transforms from the openni_tracker
    tf::TransformListener tf_listener_;

    /// publishs transforms for internal calculations and visualization
    tf::TransformBroadcaster tf_broadcaster_;
    
    /// time to wait for tf transforms in seconds
    double wait_for_tf_;
    
    /// publishers nad message for sending the pose commands for each endpoint
    ros::Publisher pub_torso_pose_;
    ros::Publisher pub_head_pose_;
    ros::Publisher pub_r_elbow_pose_;
    ros::Publisher pub_r_hand_pose_;
    ros::Publisher pub_l_elbow_pose_;
    ros::Publisher pub_l_hand_pose_;
    geometry_msgs::PoseStamped pose_;
    
    /// strings for configuring the motion adaption process
    std::string world_ref_frame_str_;
    std::string user_torso_str_;
    std::string user_head_str_;
    std::string user_r_shoulder_str_;
    std::string user_r_elbow_str_;
    std::string user_r_hand_str_;      
    std::string user_l_shoulder_str_;
    std::string user_l_elbow_str_;
    std::string user_l_hand_str_;  
    std::string robot_base_str_;
    std::string robot_ref_torso_str_;
    std::string robot_torso_str_;
    std::string robot_head_str_;
    std::string robot_r_shoulder_str_;
    std::string robot_r_elbow_str_;
    std::string robot_r_hand_str_;
    std::string robot_l_shoulder_str_;
    std::string robot_l_elbow_str_;
    std::string robot_l_hand_str_;          
    
    /// vectors for defining rotations of orientations
    btVector3 ref_frame_rot_vec_;
    btVector3 torso_goal_rot_vec_;
    btVector3 head_goal_rot_vec_;
    btVector3 r_elbow_goal_rot_vec_;
    btVector3 r_hand_goal_rot_vec_;
    btVector3 l_elbow_goal_rot_vec_;    
    btVector3 l_hand_goal_rot_vec_;
    
    tf::StampedTransform tf_usr_torso_;
    tf::StampedTransform tf_usr_head_;
    tf::StampedTransform tf_usr_r_shoulder_;
    tf::StampedTransform tf_usr_r_shoulder_elbow_;
    tf::StampedTransform tf_usr_r_shoulder_hand_;
    tf::StampedTransform tf_usr_r_elbow_;
    tf::StampedTransform tf_usr_r_elbow_hand_;
    tf::StampedTransform tf_usr_r_hand_;
    tf::StampedTransform tf_usr_l_shoulder_;
    tf::StampedTransform tf_usr_l_shoulder_elbow_;
    tf::StampedTransform tf_usr_l_shoulder_hand_;
    tf::StampedTransform tf_usr_l_elbow_;
    tf::StampedTransform tf_usr_l_elbow_hand_;
    tf::StampedTransform tf_usr_l_hand_;
    tf::StampedTransform tf_robot_ref_torso_;
    tf::StampedTransform tf_robot_torso_torso_;    
    tf::StampedTransform tf_robot_torso_head_;
    tf::StampedTransform tf_robot_torso_r_shoulder_;
    tf::StampedTransform tf_robot_torso_l_shoulder_;    
    tf::StampedTransform tf_robot_r_shoulder_l_shoulder_;    
    tf::StampedTransform tf_robot_r_shoulder_r_elbow_;
    tf::StampedTransform tf_robot_r_elbow_r_hand_;
        
    tf::StampedTransform tf_ref_frame_;
    tf::StampedTransform tf_torso_aligned_;
    tf::StampedTransform tf_torso_goal_;
    tf::StampedTransform tf_head_goal_;
    tf::StampedTransform tf_r_shoulder_scaled_;
    tf::StampedTransform tf_r_shoulder_goal_;
    tf::StampedTransform tf_r_shoulder_elbow_;
    tf::StampedTransform tf_r_shoulder_hand_;
    tf::StampedTransform tf_r_elbow_scaled_;
    tf::StampedTransform tf_r_elbow_pos_;
    tf::StampedTransform tf_r_elbow_orient_;    
    tf::StampedTransform tf_r_elbow_hand_;
    tf::StampedTransform tf_r_elbow_goal_;
    tf::StampedTransform tf_r_hand_scaled_;
    tf::StampedTransform tf_r_hand_adjusted_;      
    tf::StampedTransform tf_r_hand_goal_;
    tf::StampedTransform tf_l_shoulder_scaled_;    
    tf::StampedTransform tf_l_shoulder_goal_;
    tf::StampedTransform tf_l_shoulder_elbow_;
    tf::StampedTransform tf_l_shoulder_hand_;
    tf::StampedTransform tf_l_elbow_scaled_;
    tf::StampedTransform tf_l_elbow_pos_;
    tf::StampedTransform tf_l_elbow_orient_;        
    tf::StampedTransform tf_l_elbow_hand_;
    tf::StampedTransform tf_l_elbow_goal_;
    tf::StampedTransform tf_l_hand_scaled_;
    tf::StampedTransform tf_l_hand_goal_;

    tf::Quaternion quat_, quat_adjust_;
    
    double robot_shoulder_heigth_, robot_shoulder_width_, robot_head_height_, robot_shoulder_x_offset_;
    double robot_upper_arm_length_, robot_lower_arm_length_, robot_arm_length_;
    double user_shoulder_width_, user_shoulder_height_, user_upper_arm_length_, user_arm_length_;
    double x_norm_, y_norm_, z_norm_, x_adapt_, y_adapt_, z_adapt_, elbow_x_, elbow_y_, limb_length_;
    
    btVector3 vec_shoulder_elbow_, vec_shoulder_hand_, vec_elbow_hand_, vec_normal_, vec_helper_;
    btVector3 vec_r_shoulder_elbow_valid_, vec_l_shoulder_elbow_valid_;
    btVector3 vec_r_elbow_hand_valid_, vec_l_elbow_hand_valid_;
    bool r_elbow_extended_, l_elbow_extended_;
    btMatrix3x3 mat_orientation_;
};
