cmake_minimum_required(VERSION 3.8)
project(thorp_nav2_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/SlowEscape.action
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Slow escape behavior, for the behavior server
add_library(thorp_slow_escape_behavior SHARED src/slow_escape.cpp)
target_include_directories(thorp_slow_escape_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(thorp_slow_escape_behavior SYSTEM PUBLIC
  ${nav2_behaviors_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS}
)
target_link_libraries(thorp_slow_escape_behavior PUBLIC
  ${cpp_typesupport_target}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
)

# Its behavior tree node, for the behavior tree navigator
add_library(thorp_slow_escape_action_bt_node SHARED src/slow_escape_action.cpp)
target_compile_definitions(thorp_slow_escape_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(thorp_slow_escape_action_bt_node SYSTEM PRIVATE ${nav2_behavior_tree_INCLUDE_DIRS})
target_link_libraries(thorp_slow_escape_action_bt_node PRIVATE
  ${cpp_typesupport_target}
  ${nav2_behavior_tree_LIBRARIES}
  behaviortree_cpp::behaviortree_cpp
  nav2_util::nav2_util_core
  rclcpp::rclcpp
)

pluginlib_export_plugin_description_file(nav2_core plugins/slow_escape.xml)

install(TARGETS thorp_slow_escape_behavior thorp_slow_escape_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
