#include <algorithm>
#include <string>
#include <vector>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

class TrajectoryPublisher
{
public:
  TrajectoryPublisher()
   : approach_time_(3.0), // NOTE: Magic value
     approach_count_(0)
  {
    controlled_joints_.push_back("torso_1_joint");
    controlled_joints_.push_back("torso_2_joint");
    controlled_joints_.push_back("head_1_joint");
    controlled_joints_.push_back("head_2_joint");
    controlled_joints_.push_back("arm_left_1_joint");
    controlled_joints_.push_back("arm_left_2_joint");
    controlled_joints_.push_back("arm_left_3_joint");
    controlled_joints_.push_back("arm_left_4_joint");
    controlled_joints_.push_back("arm_left_5_joint");
    controlled_joints_.push_back("arm_left_6_joint");
    controlled_joints_.push_back("arm_left_7_joint");
    controlled_joints_.push_back("arm_right_1_joint");
    controlled_joints_.push_back("arm_right_2_joint");
    controlled_joints_.push_back("arm_right_3_joint");
    controlled_joints_.push_back("arm_right_4_joint");
    controlled_joints_.push_back("arm_right_5_joint");
    controlled_joints_.push_back("arm_right_6_joint");
    controlled_joints_.push_back("arm_right_7_joint");

    loop_rate_ = 2; // TODO: Get from param server

    ros::NodeHandle nh;
    traj_publisher_          = nh.advertise<trajectory_msgs::JointTrajectory>("/command", 1);
    joint_states_subscriber_ = nh.subscribe("/joint_states_cmd",
                                            1, // Buffer size
                                            &TrajectoryPublisher::publishTrajectory, this);
  }
private:
  std::vector<std::string> controlled_joints_;
  ros::Publisher  traj_publisher_;
  ros::Subscriber joint_states_subscriber_;
  int loop_rate_;
  double approach_time_;
  int approach_count_;

void publishTrajectory(const sensor_msgs::JointState& joint_states_cmd)
{
  const double current_approach_time = approach_time_ - static_cast<double>(approach_count_) / static_cast<double>(loop_rate_);
  ros::Duration approach_delta;
  if (current_approach_time > 0.0)
  {
    approach_delta = ros::Duration(current_approach_time);
    ++approach_count_;
  }
  else
  {
    approach_delta = ros::Duration(0.0);
  }

  trajectory_msgs::JointTrajectory traj;
  const ros::Duration delta(0.2 / loop_rate_); // NOTE: Magic value

  traj.header.stamp = joint_states_cmd.header.stamp + delta;
  traj.points.resize(1);

  for(unsigned int k = 0; k < joint_states_cmd.name.size(); ++k)
  {
    if(std::find(controlled_joints_.begin(), controlled_joints_.end(), joint_states_cmd.name[k] ) != controlled_joints_.end())
    {
      traj.joint_names.push_back(joint_states_cmd.name[k]);
      traj.points.front().positions.push_back(joint_states_cmd.position[k]);
      traj.points.front().velocities.push_back(joint_states_cmd.velocity[k]);
    }
  }
  traj.points.front().time_from_start = ros::Duration(1.0 / loop_rate_) + delta + approach_delta;
  traj_publisher_.publish(traj);
}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reem_traj_publisher");
  TrajectoryPublisher traj_publisher;
  ros::Rate loopRate(10);

  // Main loop
  while (ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  return EXIT_SUCCESS;
}