cmake_minimum_required(VERSION 2.8.3)
project(motion_retargeting_controller)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        tf
                                        tf_conversions
                                        geometry_msgs
                                        sensor_msgs
                                        trajectory_msgs
                                        motion_adaption
                                        tree_kinematics)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(motion_retargeting                  src/motion_retargeting.cpp)
#add_library(motion_retargeting_configuration    src/motion_retargeting_configuration.cpp)
add_executable(motion_retargeting_controller    src/motion_retargeting_controller.cpp)

#target_link_libraries(motion_retargeting_controller motion_retargeting motion_retargeting_configuration)
target_link_libraries(motion_retargeting_controller motion_retargeting)
target_link_libraries(motion_retargeting_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS motion_retargeting_controller
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})