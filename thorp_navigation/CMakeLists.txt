cmake_minimum_required(VERSION 2.8.3)
project(thorp_navigation)

find_package(catkin REQUIRED roscpp actionlib geometry_msgs mbf_msgs thorp_msgs thorp_toolkit
                             dynamic_reconfigure visualization_msgs visp_ros visp_bridge)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/Follower.cfg)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS roscpp actionlib geometry_msgs mbf_msgs thorp_msgs thorp_toolkit
            dynamic_reconfigure visualization_msgs visp_ros visp_bridge)

# nodes
add_executable(pose_servoing src/pose_servoing.cpp)
target_link_libraries(pose_servoing ${catkin_LIBRARIES})
add_dependencies(pose_servoing thorp_msgs_generate_messages_cpp)

add_executable(pose_follower src/pose_follower.cpp)
target_link_libraries(pose_follower ${catkin_LIBRARIES})
add_dependencies(pose_follower thorp_msgs_generate_messages_cpp)

add_executable(waypoints_path src/waypoints_path.cpp)
target_link_libraries(waypoints_path ${catkin_LIBRARIES})
add_dependencies(waypoints_path thorp_msgs_generate_messages_cpp)
