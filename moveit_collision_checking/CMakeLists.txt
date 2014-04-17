cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_checking)

find_package(catkin REQUIRED COMPONENTS motion_retargeting
                                        moveit_core
                                        moveit_ros_planning
                                        roscpp)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS motion_retargeting
                              moveit_core
                              moveit_ros_planning
                              roscpp)

include_directories(include
                    ${catkin_INCLUDE_DIRS})
#link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/moveit_collision_checker.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
 
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
