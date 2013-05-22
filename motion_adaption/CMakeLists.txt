cmake_minimum_required(VERSION 2.8.3)
project(motion_adaption)

find_package(eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        tf
                                        tf_conversions
                                        geometry_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS roscpp
                              tf
                              tf_conversions
                              geometry_msgs
               DEPENDS eigen
                       Boost)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_library(motion_adaption src/motion_adaption.cpp)
target_link_libraries(motion_adaption ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_motion_adaption src/test.cpp)
target_link_libraries(test_motion_adaption motion_adaption)


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
