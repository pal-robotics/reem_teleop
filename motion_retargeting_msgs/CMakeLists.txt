cmake_minimum_required(VERSION 2.8.3)
project(motion_retargeting_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES Motion.msg
                        MotionList.msg
                        MotionPlayback.msg
                        PlaybackStatus.msg
                        PlaybackStatusCodes.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES utils
               CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(utils src/utils.cpp)
target_link_libraries(utils ${catkin_LIBRARIES})
add_dependencies(utils ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS utils
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})