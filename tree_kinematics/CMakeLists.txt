cmake_minimum_required(VERSION 2.8.3)
project(tree_kinematics)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED) 
find_package(catkin REQUIRED COMPONENTS roscpp
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
                              kdl_parser
                              tf
                              kdl_conversions
                              urdf
                              std_msgs
                              geometry_msgs
                              sensor_msgs
                              message_runtime
               DEPENDS Boost
                       orocos_kdl)

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(tree_kinematics src/tree_kinematics.cpp
                            src/treeiksolverpos_online.cpp)
target_link_libraries(tree_kinematics ${Boost_LIBRARIES}
                                      ${catkin_LIBRARIES}
                                      ${orocos_kdl_LIBRARIES})
add_dependencies(tree_kinematics tree_kinematics_gencpp)

add_executable(tree_kinematics_node src/tree_kinematics_node.cpp)
target_link_libraries(tree_kinematics_node tree_kinematics)

install(TARGETS tree_kinematics
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
install(TARGETS tree_kinematics_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
