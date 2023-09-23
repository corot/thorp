cmake_minimum_required(VERSION 3.10)
project(thorp_cannon)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    arbotix_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
    LIBRARIES
        particle_shooter_plugin
    DEPENDS
        roscpp
        gazebo_ros
        arbotix_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For particle_shooter_plugin plugin
add_library(gazebo_cannon_plugin src/gazebo_cannon_plugin.cpp)
add_dependencies(gazebo_cannon_plugin arbotix_msgs_generate_messages_cpp)
target_link_libraries(gazebo_cannon_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(PROGRAMS nodes/cannon_ctrl.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
