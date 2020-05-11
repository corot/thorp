cmake_minimum_required(VERSION 2.8.3)
project(thorp_navigation)

find_package(catkin REQUIRED roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros visp_bridge)

link_directories(${catkin_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                   )

catkin_package(DEPENDS roscpp actionlib geometry_msgs thorp_msgs thorp_toolkit visp_ros)

# nodes
add_executable(pose_servoing src/pose_servoing.cpp)
target_link_libraries(pose_servoing ${catkin_LIBRARIES})
add_dependencies(pose_servoing thorp_msgs_generate_messages_cpp)
