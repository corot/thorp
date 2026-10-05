cmake_minimum_required(VERSION 3.8)
project(thorp_bt_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(thorp_costmap_layers REQUIRED)
find_package(thorp_msgs REQUIRED)
find_package(thorp_toolkit REQUIRED)

# Custom behavior tree nodes; listed one by one, as the package is ported in stages
set(BT_NODES
  src/bt/actions/cannon_control.cpp
  src/bt/actions/clear_list.cpp
  src/bt/actions/clear_octomap.cpp
  src/bt/actions/clear_planning_scene.cpp
  src/bt/actions/clear_table_access.cpp
  src/bt/actions/detach_object.cpp
  src/bt/actions/detect_objects.cpp
  src/bt/actions/detect_tables.cpp
  src/bt/actions/drag_and_drop.cpp
  src/bt/actions/expand_pickup_location.cpp
  src/bt/actions/follow_object.cpp
  src/bt/actions/get_closest_pose.cpp
  src/bt/actions/get_from_list.cpp
  src/bt/actions/get_list_front.cpp
  src/bt/actions/get_poses_around_table.cpp
  src/bt/actions/get_robot_pose.cpp
  src/bt/actions/list_slicing.cpp
  src/bt/actions/look_at_pose.cpp
  src/bt/actions/monitor_target.cpp
  src/bt/actions/move_gripper.cpp
  src/bt/actions/next_pose_on_tray.cpp
  src/bt/actions/pickup_object.cpp
  src/bt/actions/place_object.cpp
  src/bt/actions/plan_pickup_locations.cpp
  src/bt/actions/plan_room_exploration.cpp
  src/bt/actions/plan_room_sequence.cpp
  src/bt/actions/pop_from_list.cpp
  src/bt/actions/pose_as_path.cpp
  src/bt/actions/publish_pose.cpp
  src/bt/actions/push_to_list.cpp
  src/bt/actions/read_ros_param.cpp
  src/bt/actions/read_user_commands.cpp
  src/bt/actions/record_pickup_failure.cpp
  src/bt/actions/restore_table_access.cpp
  src/bt/actions/segment_rooms.cpp
  src/bt/actions/select_next_target.cpp
  src/bt/actions/set_arm_config.cpp
  src/bt/actions/set_blackboard.cpp
  src/bt/actions/settle_on_tray.cpp
  src/bt/actions/table_as_obstacle.cpp
  src/bt/actions/track_progress.cpp
  src/bt/actions/transform_pose.cpp
  src/bt/conditions/are_same_pose.cpp
  src/bt/conditions/camera_fov_clear.cpp
  src/bt/conditions/cannon_has_ammo.cpp
  src/bt/conditions/gripper_busy.cpp
  src/bt/conditions/in_blackboard.cpp
  src/bt/conditions/is_bool_true.cpp
  src/bt/conditions/is_list_empty.cpp
  src/bt/conditions/object_attached.cpp
  src/bt/conditions/objects_detected.cpp
  src/bt/conditions/table_valid_size.cpp
  src/bt/conditions/table_visited.cpp
  src/bt/conditions/target_reachable.cpp
  src/bt/conditions/tray_full.cpp
  src/bt/controls/recover_and_retry.cpp
  src/bt/decorators/for_each.cpp
  src/bt/decorators/store_result.cpp
)

# The nodes register themselves on static initialization, into the runner's factory
add_library(bt_runner SHARED src/bt_runner.cpp src/bt_ros_logger.cpp ${BT_NODES})
target_include_directories(bt_runner PUBLIC include ${nav2_behavior_tree_INCLUDE_DIRS})
target_link_libraries(bt_runner PUBLIC
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  behaviortree_cpp::behaviortree_cpp
  ${nav2_behavior_tree_LIBRARIES}
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_planning_scene_interface
  ${control_msgs_TARGETS}
  interactive_markers::interactive_markers
  ${nav_msgs_TARGETS}
  ${rviz_2d_overlay_msgs_TARGETS}
  tinyxml2::tinyxml2
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${visualization_msgs_TARGETS}
  ${thorp_costmap_layers_TARGETS}
  ${thorp_msgs_TARGETS}
  thorp_toolkit::thorp_toolkit
)

add_executable(bt_runner_node src/bt_runner_node.cpp)
target_link_libraries(bt_runner_node bt_runner)

add_executable(bt_server_node src/bt_server_node.cpp src/bt_server.cpp src/blackboard_json.cpp)
target_link_libraries(bt_server_node bt_runner nlohmann_json::nlohmann_json)

install(TARGETS bt_runner
        LIBRARY DESTINATION lib
)

install(TARGETS bt_runner_node bt_server_node
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/show_bt_node_on_rviz.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/bt_runner.launch.py launch/bt_server.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

# What each tree offers as a capability, for bt_server's callers
install(FILES config/capabilities.yaml
        DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY bt
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
