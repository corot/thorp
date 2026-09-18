cmake_minimum_required(VERSION 3.10)
project(thorp_bt_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PKG_DEPS
      tf2
      roscpp
      actionlib
      geometry_msgs
      kobuki_msgs
      mbf_msgs
      nav_msgs
      std_msgs
      std_srvs
      jsk_rviz_plugins
      ipa_building_msgs
      cob_perception_msgs
      rail_manipulation_msgs
      behaviortree_cpp
      thorp_msgs
      thorp_toolkit
      thorp_costmap_layers
)

find_package(catkin REQUIRED
  COMPONENTS ${PKG_DEPS}
)

find_package(Boost COMPONENTS thread chrono exception REQUIRED)
# No nlohmann_json here on purpose: we use the copy vendored by behaviortree_cpp (see the
# note in blackboard_json.hpp), because two nlohmann versions in one build silently fail to
# link against each other.

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bt_runner
  CATKIN_DEPENDS ${PKG_DEPS}
  DEPENDS Boost
)

include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE BT_NODES
        src/bt/actions/*.cpp
        src/bt/conditions/*.cpp
        src/bt/controls/*.cpp
        src/bt/decorators/*.cpp)

add_library(bt_runner src/bt_runner.cpp src/bt_ros_logger.cpp ${BT_NODES})
target_link_libraries(bt_runner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(bt_runner_node src/bt_runner_node.cpp)
add_dependencies(bt_runner_node bt_runner)
target_link_libraries(bt_runner_node bt_runner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# bt_server: loads every tree under bt/ and runs whichever one is requested through the
# RunSubtree action. Links against bt_runner to reuse its statically-registered node
# factory (Runner::bt_factory_), so every custom node stays available without duplicating
# their registration.
add_library(bt_server src/bt_server.cpp src/blackboard_json.cpp)
target_link_libraries(bt_server
  bt_runner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(bt_server_node src/bt_server_node.cpp)
add_dependencies(bt_server_node bt_server)
target_link_libraries(bt_server_node bt_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS bt_runner bt_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bt_runner_node bt_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
