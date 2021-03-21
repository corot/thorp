cmake_minimum_required(VERSION 2.8.3)
project(thorp_manipulation)

find_package(catkin REQUIRED roscpp actionlib geometry_msgs moveit_msgs moveit_core visualization_msgs
                             interactive_markers gazebo_grasp_plugin thorp_msgs thorp_toolkit)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    SYSTEM
                    ${catkin_INCLUDE_DIRS})

catkin_package(DEPENDS roscpp actionlib geometry_msgs moveit_msgs moveit_core visualization_msgs
                       interactive_markers gazebo_grasp_plugin thorp_msgs thorp_toolkit)


# nodes
add_executable(thorp_arm_ctrl_server src/thorp_arm_controller.cpp
                                     src/pickup_object_server.cpp
                                     src/place_object_server.cpp
                                     src/move_to_target_server.cpp
                                     src/house_keeping_server.cpp
                                     src/joint_state_watchdog.cpp
                                     src/thorp_arm_ctrl_server.cpp)
target_link_libraries(thorp_arm_ctrl_server ${catkin_LIBRARIES})
add_dependencies(thorp_arm_ctrl_server thorp_msgs_generate_messages_cpp)

add_executable(scene_interaction_server src/interactive_manip_server.cpp)
target_link_libraries(scene_interaction_server ${catkin_LIBRARIES})
add_dependencies(scene_interaction_server thorp_msgs_generate_messages_cpp)
