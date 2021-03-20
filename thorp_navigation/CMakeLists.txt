cmake_minimum_required(VERSION 2.8.3)
project(thorp_navigation)

find_package(catkin REQUIRED roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros visp_bridge
                             turtlebot_msgs dynamic_reconfigure visualization_msgs costmap_2d base_local_planner)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                   )

generate_dynamic_reconfigure_options(cfg/Follower.cfg)

catkin_package(DEPENDS roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros visp_bridge
                       turtlebot_msgs dynamic_reconfigure visualization_msgs costmap_2d base_local_planner)

# nodes
add_executable(pose_servoing src/pose_servoing.cpp)
target_link_libraries(pose_servoing ${catkin_LIBRARIES})
add_dependencies(pose_servoing thorp_msgs_generate_messages_cpp)

add_executable(pose_follower src/pose_follower.cpp)
target_link_libraries(pose_follower ${catkin_LIBRARIES})
add_dependencies(pose_follower thorp_msgs_generate_messages_cpp)

add_executable(path_smoother src/path_smoother.cpp)
target_link_libraries(path_smoother ${catkin_LIBRARIES})
add_dependencies(path_smoother thorp_msgs_generate_messages_cpp)
