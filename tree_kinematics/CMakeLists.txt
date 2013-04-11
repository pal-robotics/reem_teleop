cmake_minimum_required(VERSION 2.8.3)
project(tree_kinematics)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        orocos_kdl
                                        kdl_parser
                                        tf
                                        kdl_conversions
                                        urdf
                                        std_msgs
                                        geometry_msgs
                                        sensor_msgs
                                        message_generation)

add_message_files(FILES JointLimits.msg
                        KinematicSolverInfo.msg)

add_service_files(FILES GetTreePositionIK.srv
                        GetPositionFK.srv)

generate_messages(DEPENDENCIES std_msgs
                               geometry_msgs
                               sensor_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS roscpp
                              orocos_kdl
                              kdl_parser
                              tf
                              kdl_conversions
                              urdf
                              std_msgs
                              geometry_msgs
                              sensor_msgs
                              message_runtime
               DEPENDS Boost)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(tree_kinematics src/tree_kinematics.cpp
                            src/treeiksolverpos_online.cpp)
target_link_libraries(tree_kinematics ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(tree_kinematics tree_kinematics_gencpp)

add_executable(tree_kinematics_node src/tree_kinematics_node.cpp)
target_link_libraries(tree_kinematics_node tree_kinematics)

install(TARGETS tree_kinematics
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
install(TARGETS tree_kinematics_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_reem_teleop.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
