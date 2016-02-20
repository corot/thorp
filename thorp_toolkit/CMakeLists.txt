cmake_minimum_required(VERSION 2.8.3)
project(thorp_toolkit)

##############################################################################
# Find Packages
##############################################################################

find_package(Boost REQUIRED COMPONENTS thread)
 
find_package(catkin
    REQUIRED
    COMPONENTS
        geometry_msgs
        roscpp
        tf
)


##############################################################################
# Exports
##############################################################################

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
        tf
)


##############################################################################
# Project Configuration
##############################################################################

add_definitions(${catkin_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})    


##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)


##############################################################################
# Build
##############################################################################

## Nodes
add_executable(save_pose_node nodes/save_pose_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(save_pose_node
  ${catkin_LIBRARIES}
)


##############################################################################
# Install
##############################################################################

## TODO ##