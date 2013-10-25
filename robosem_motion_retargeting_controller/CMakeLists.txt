cmake_minimum_required(VERSION 2.8.3)
project(robosem_motion_retargeting_controller)

find_package(catkin REQUIRED COMPONENTS motion_adaption
                                        motion_retargeting
                                        roscpp
                                        rosbag
                                        sensor_msgs
                                        tree_kinematics)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(robosem_mo_ret_controller src/robosem_mo_ret_controller.cpp)
target_link_libraries(robosem_mo_ret_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robosem_motion_playback src/robosem_motion_playback.cpp)
target_link_libraries(robosem_motion_playback ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS robosem_mo_ret_controller
                robosem_motion_playback
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})