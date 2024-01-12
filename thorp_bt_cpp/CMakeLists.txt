cmake_minimum_required(VERSION 3.10)
project(thorp_bt_cpp)

set(PKG_DEPS
      tf2
      roscpp
      actionlib
      geometry_msgs
      mbf_msgs
      nav_msgs
      std_msgs
      std_srvs
      ipa_building_msgs
      cob_perception_msgs
      behaviortree_cpp_v3
      thorp_toolkit
)

find_package(catkin REQUIRED
  COMPONENTS ${PKG_DEPS}
)

find_package(Boost COMPONENTS thread chrono exception REQUIRED)

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

add_library(bt_runner src/bt_runner.cpp)
target_link_libraries(bt_runner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(bt_runner_node src/bt_runner_node.cpp ${BT_NODES})
add_dependencies(bt_runner_node bt_runner)
target_link_libraries(bt_runner_node bt_runner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS bt_runner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bt_runner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
