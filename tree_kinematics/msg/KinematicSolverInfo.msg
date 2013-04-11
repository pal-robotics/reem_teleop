### Copy of kinematics_msgs/KinematicSolverInfo.msg - used since arm_navigation hasn't been catkinized yet! ###
# A list of joints in the kinematic tree
string[] joint_names
# A list of joint limits corresponding to the joint names
JointLimits[] limits
# A list of links that the kinematics node provides solutions for
string[] link_names
