cmake_minimum_required(VERSION 2.8.3)
project(motion_retargeting)

find_package(catkin REQUIRED COMPONENTS actionlib
                                        motion_adaption 
                                        rosbag
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf
                                        tree_kinematics)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS motion_adaption
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


install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
