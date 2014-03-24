cmake_minimum_required(VERSION 2.8.3)
project(motion_retargeting)

find_package(catkin REQUIRED COMPONENTS actionlib
                                        control_msgs
                                        motion_adaption
                                        motion_retargeting_msgs
                                        rosbag
                                        roscpp
                                        roslib
                                        sensor_msgs
                                        std_msgs
                                        tf
                                        tree_kinematics)

find_package(Boost REQUIRED COMPONENTS filesystem
                                       regex)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS actionlib
                              control_msgs
                              motion_adaption
                              rosbag
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf
                              tree_kinematics
               DEPENDS system_lib)

include_directories(include
                    ${catkin_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/motion_retargeting.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}
                                      ${catkin_LIBRARIES})

add_executable(motion_player_rosbag_fjta src/motion_player_rosbag_fjta.cpp)
target_link_libraries(motion_player_rosbag_fjta ${catkin_LIBRARIES}
                                                ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
