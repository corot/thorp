cmake_minimum_required(VERSION 2.8.3)
project(thorp_navigation)

find_package(catkin REQUIRED roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros visp_bridge
                             turtlebot_msgs dynamic_reconfigure)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                   )

generate_dynamic_reconfigure_options(cfg/Follower.cfg)

catkin_package(DEPENDS roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros
                       turtlebot_msgs dynamic_reconfigure)

# nodelets
add_library(pose_follower src/pose_follower.cpp)
target_link_libraries(pose_follower ${catkin_LIBRARIES})
add_dependencies(pose_follower thorp_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp)

# nodes
add_executable(pose_servoing src/pose_servoing.cpp)
target_link_libraries(pose_servoing ${catkin_LIBRARIES})
add_dependencies(pose_servoing thorp_msgs_generate_messages_cpp)

# TODO (if ever needed)
# add_executable(pose_follower_node nodes/pose_follower.cpp)
# target_link_libraries(pose_follower_node pose_follower)
