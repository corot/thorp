cmake_minimum_required(VERSION 2.8.3)
project(thorp_toolkit)

## Find catkin macros and libraries
find_package(catkin REQUIRED
	geometry_msgs
	roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Declare a cpp executable
add_executable(save_pose_node nodes/save_pose_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(save_pose_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## TODO ##