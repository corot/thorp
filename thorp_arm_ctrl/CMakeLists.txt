cmake_minimum_required(VERSION 2.8.3)
project(thorp_arm_ctrl)

find_package(catkin REQUIRED roscpp actionlib actionlib_msgs arbotix_msgs geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface
                             thorp_msgs yocs_math_toolkit geometric_shapes interactive_markers visualization_msgs)
                             
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    SYSTEM
                    ${catkin_INCLUDE_DIRS}
                   )

catkin_package(DEPENDS roscpp actionlib actionlib_msgs arbotix_msgs geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface
                       thorp_msgs yocs_math_toolkit geometric_shapes interactive_markers visualization_msgs)


# nodes
add_executable(thorp_arm_ctrl_server src/pickup_object_server.cpp
                                     src/place_object_server.cpp
                                     src/move_to_target_server.cpp
                                     src/thorp_arm_ctrl_server.cpp)
target_link_libraries(thorp_arm_ctrl_server ${catkin_LIBRARIES})
add_dependencies(thorp_arm_ctrl_server thorp_msgs_gencpp)

add_executable(scene_interaction_server src/interactive_manip_server.cpp)
target_link_libraries(scene_interaction_server ${catkin_LIBRARIES})
add_dependencies(scene_interaction_server thorp_msgs_gencpp)
