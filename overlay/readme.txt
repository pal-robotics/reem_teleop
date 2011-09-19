The "overlay.tar.gz" file contains the three packages "kdl_parser", "pr2_mechanism_model" and "tf_conversions", as well as the stack "orocos_kinematics_dynamics". All of them need to overlay existing packages/stacks of your current version of ros.
All of this is necessary in order to enable the use of the new "orocos-kdl" located in the orocos_kinematics_dynamics stack.
To overlay your current packages/stacks extract the compressed container to your desired destinations and add the their paths to the ROS_PACKAGE_PATH. Make sure they are placed in front of the path(s) to the original packages/stacks. Use "rospack find" + package name in order to check, if the overlaying is working.

27.06.2011 - Marcus Liebhardt
