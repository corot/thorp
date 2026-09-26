cmake_minimum_required(VERSION 3.8)
project(thorp_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(thorp_msgs REQUIRED)

# Pickup and place object action servers
add_executable(manipulation_node src/manipulation_node.cpp
                                 src/pick_and_place_server.cpp
                                 src/arm_targets.cpp)
target_include_directories(manipulation_node PRIVATE include)
target_link_libraries(manipulation_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  moveit_core::moveit_robot_model
  moveit_core::moveit_planning_scene
  moveit_core::moveit_utils
  ${moveit_msgs_TARGETS}
  moveit_ros_planning::moveit_robot_model_loader
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
  ${moveit_task_constructor_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  ${thorp_msgs_TARGETS}
)

install(TARGETS manipulation_node
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS nodes/fake_joint_pub.py
                 scripts/test_pick_and_place.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/manipulation.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES launch/includes/arm.launch.py
        DESTINATION share/${PROJECT_NAME}/launch/includes
)

install(FILES param/pick_and_place.yaml
              param/pick_and_place_gazebo.yaml
        DESTINATION share/${PROJECT_NAME}/param
)

ament_package()
