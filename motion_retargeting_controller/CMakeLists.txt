cmake_minimum_required(VERSION 2.8.3)
project(motion_retargeting_controller)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(catkin REQUIRED COMPONENTS actionlib
                                        rosbag
                                        roscpp
                                        tf
                                        tf_conversions
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        trajectory_msgs
                                        motion_adaption
                                        tree_kinematics)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(motion_retargeting                  src/motion_retargeting.cpp)

add_executable(robosem_mo_ret_controller            src/robosem_mo_ret_controller.cpp)
target_link_libraries(robosem_mo_ret_controller motion_retargeting)
target_link_libraries(robosem_mo_ret_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robosem_motion_playback    src/robosem_motion_playback.cpp)
target_link_libraries(robosem_motion_playback motion_retargeting)
target_link_libraries(robosem_motion_playback ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS motion_retargeting_controller
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})