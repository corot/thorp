cmake_minimum_required(VERSION 2.8.3)
project(thorp_mbf_plugins)

# Required to use auto variables and to_string
add_definitions("-std=c++11 -W")

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        pluginlib
        costmap_2d
        mbf_msgs
        mbf_costmap_core
        base_local_planner
        thorp_toolkit
        )

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES slow_escape_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        mbf_msgs
        mbf_costmap_core
        base_local_planner
        thorp_toolkit
)

add_library(slow_escape_recovery
    src/slow_escape_recovery.cpp)
target_link_libraries(slow_escape_recovery
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(TARGETS slow_escape_recovery
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES plugins/slow_escape_recovery.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
