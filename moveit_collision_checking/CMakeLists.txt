cmake_minimum_required(VERSION 2.8.3)
project(moveit_collision_checking)

find_package(catkin REQUIRED COMPONENTS motion_retargeting
                                        moveit_core
                                        moveit_ros_planning
                                        roscpp)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS motion_retargeting
                              moveit_core
                              moveit_ros_planning
                              roscpp)
 
#include_directories(include
#                    ${catkin_INCLUDE_DIRS})
#link_directories(${orocos_kdl_LIBRARY_DIRS})

#add_library(moveit_collision_checking
#            src/${PROJECT_NAME}/moveit_collision_checking.cpp)

#add_executable(moveit_collision_checking src/moveit_collision_checking.cpp)

#target_link_libraries(moveit_collision_checking ${catkin_LIBRARIES})

#install(TARGETS moveit_collision_checking
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
