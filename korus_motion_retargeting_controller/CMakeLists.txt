cmake_minimum_required(VERSION 2.8.3)
project(korus_motion_retargeting_controller)

find_package(catkin REQUIRED COMPONENTS motion_adaption
                                        motion_retargeting
                                        roscpp
                                        tree_kinematics)

catkin_package()

#include_directories(${catkin_INCLUDE_DIRS})
#link_directories(${orocos_kdl_LIBRARY_DIRS})

#add_executable(korus_mo_ret_controller src/korus_mo_ret_controller.cpp)
#target_link_libraries(korus_mo_ret_controller ${catkin_LIBRARIES})

#install(TARGETS korus_mo_ret_controller
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})