cmake_minimum_required(VERSION 2.8.3)
project(tree_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        orocos_kdl
                                        kdl_parser
                                        tf
                                        tf_conversions
                                        urdf
                                        geometry_msgs
                                        kinematics_msgs
                                        arm_navigation_msgs
                                        message_generation)

add_service_files(FILES get_tree_position_ik.srv)

generate_messages(DEPENDENCIES kinematics_msgs
                               arm_navigation_msgs)

# extend this if we want to make the library available outside the package
catkin_package(CATKIN_DEPENDS message_runtime kinematics_msgs arm_navigation_msgs)

include_directories(include/tree_kinematics ${catkin_INCLUDE_DIRS})

add_library(tree_kinematics src/tree_kinematics.cpp
                            src/treeiksolverpos_online.cpp)

add_executable(tree_kinematics_node src/tree_kinematics_node.cpp)
target_link_libraries(tree_kinematics_node tree_kinematics)

target_link_libraries(tree_kinematics_node ${catkin_LIBRARIES})

install(TARGETS tree_kinematics tree_kinematics_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
