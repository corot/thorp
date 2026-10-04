cmake_minimum_required(VERSION 3.8)
project(thorp_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# PCL's find modules use <package>_ROOT variables, also on their own policy scopes
cmake_policy(SET CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree registration sample_consensus search
                                     segmentation surface)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(thorp_msgs REQUIRED)
find_package(thorp_toolkit REQUIRED)

# Tables and tabletop objects detection
add_executable(object_detection src/object_detection_node.cpp
                                src/segmentation.cpp
                                src/template_matcher.cpp)
target_include_directories(object_detection PRIVATE include)
target_include_directories(object_detection SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})
target_link_libraries(object_detection
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  geometric_shapes::geometric_shapes
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  moveit_ros_planning_interface::moveit_planning_scene_interface
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${PCL_LIBRARIES}
  nlohmann_json::nlohmann_json
  OpenMP::OpenMP_CXX
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${thorp_msgs_TARGETS}
  thorp_toolkit::thorp_toolkit
)

install(TARGETS object_detection
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS nodes/target_tracker.py nodes/xtion_fov_analyzer.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/perception.launch.py launch/target_detection.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES config/object_detection.yaml
        DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.pcd" PATTERN "*.stl"
        PATTERN "blender" EXCLUDE
)

ament_package()
