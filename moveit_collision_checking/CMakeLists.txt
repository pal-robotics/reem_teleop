cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_checking)

find_package(catkin REQUIRED COMPONENTS motion_retargeting
                                        moveit_core
                                        moveit_ros_planning
                                        roscpp)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS motion_retargeting
                              moveit_core
                              moveit_ros_planning
                              roscpp)
 
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
