cmake_minimum_required(VERSION 3.10)
project(thorp_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/clear_costmap_tool.hpp
  include/${PROJECT_NAME}/user_commands_panel.hpp
  src/clear_costmap_tool.cpp
  src/user_commands_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pluginlib::pluginlib
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  ${nav2_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

ament_package()
