cmake_minimum_required(VERSION 3.10)
project(thorp_bt_cpp)

set(PKG_DEPS
      angles
      dynamic_reconfigure
      tf
      tf2
      tf2_eigen
      tf2_ros
      roscpp
      mbf_utility
      actionlib
      actionlib_msgs
      geometry_msgs
      mbf_msgs
      nav_msgs
      std_msgs
      std_srvs
      ipa_building_msgs
      cob_perception_msgs
      costmap_2d
      behaviortree_cpp_v3
      thorp_toolkit
)

set(LIBS
      bt_nodes
      bt_runner
)

find_package(catkin REQUIRED
  COMPONENTS ${PKG_DEPS}
)
#
#generate_dynamic_reconfigure_options(
#  cfg/NASDispatcher.cfg
#)

find_package(Boost COMPONENTS thread chrono exception REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBS}
  CATKIN_DEPENDS ${PKG_DEPS}
  DEPENDS Boost
)

include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


file(GLOB_RECURSE BT_NODES include/thorp_bt_cpp/bt/actions/*.cpp)
MESSAGE(${BT_NODES})
add_library(bt_nodes ${BT_NODES})
set_target_properties(bt_nodes PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(bt_nodes
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(bt_runner src/bt_runner.cpp)
add_dependencies(bt_runner bt_nodes)
target_link_libraries(bt_runner bt_nodes
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_executable(bt_runner_node src/bt_runner_node.cpp /home/jorge/catkin_ws/thorp/src/thorp/thorp_bt_cpp/include/thorp_bt_cpp/bt/actions/create_pose_stamped.hpp /home/jorge/catkin_ws/thorp/src/thorp/thorp_bt_cpp/include/thorp_bt_cpp/bt/actions/go_to_pose.hpp)
add_executable(bt_runner_node src/bt_runner_node.cpp src/bt_runner.cpp ${BT_NODES}) # include/thorp_bt_cpp/bt/actions/create_pose_stamped.hpp)
#add_dependencies(bt_runner_node ${LIBS})
target_link_libraries(bt_runner_node #bt_runner bt_nodes
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${LIBS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  bt_runner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
