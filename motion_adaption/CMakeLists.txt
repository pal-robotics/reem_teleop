cmake_minimum_required(VERSION 2.8.3)
project(motion_adaption)

find_package(eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        tf
                                        tf_conversions
                                        geometry_msgs)

# extend this if we want to make the library available outside the package
catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS roscpp
                              tf
                              tf_conversions
                              geometry_msgs
               DEPENDS eigen
                       Boost)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_library(motion_adaption src/motion_adaption.cpp
#                            src/get_transforms.cpp
#                            src/set_ref_frame.cpp
#                            src/adapt_transforms.cpp
#                            src/set_goals.cpp
#                            src/publish_msgs.cpp)
add_library(motion_adaption src/motion_adaption.cpp)
#add_executable(motion_adaption_node src/motion_adaption_node.cpp)
#add_executable(motion_adaption src/motion_adaption.cpp)
#target_link_libraries(motion_adaption_node motion_adaption)
#target_link_libraries(motion_adaption_node ${catkin_LIBRARIES})
target_link_libraries(motion_adaption ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS motion_adaption
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
#install(TARGETS motion_adaption_node
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
