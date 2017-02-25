cmake_minimum_required(VERSION 2.8.3)
project(sensor_board)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )

add_service_files(
        FILES
        record_msg.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sensor_board
        CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)





set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
