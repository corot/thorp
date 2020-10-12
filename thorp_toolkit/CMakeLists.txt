cmake_minimum_required(VERSION 2.8.3)
project(thorp_toolkit)

##############################################################################
# Find Packages
##############################################################################

find_package(Boost REQUIRED COMPONENTS thread)
 
find_package(catkin
    REQUIRED
    COMPONENTS
        tf
        tf2
        tf2_ros
        tf2_geometry_msgs
        roscpp
        geometry_msgs
        geometric_shapes
        moveit_msgs
        moveit_ros_planning_interface
        eigen_conversions
        thorp_msgs
)



##############################################################################
# Exports
##############################################################################

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        tf
        tf2
        tf2_ros
        tf2_geometry_msgs
        roscpp
        geometry_msgs
        geometric_shapes
        moveit_msgs
        moveit_ros_planning_interface
        eigen_conversions
        thorp_msgs
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