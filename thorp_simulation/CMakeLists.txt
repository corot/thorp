cmake_minimum_required(VERSION 3.8)
project(thorp_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(thorp_toolkit REQUIRED)

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  thorp_toolkit::thorp_toolkit
)

install(TARGETS gazebo_ground_truth
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/spawn_gazebo_models.py nodes/cats_controller.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/thorp_gazebo.launch.py launch/navigation.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES param/gz_bridge.yaml param/cats_bridge.yaml param/controllers.yaml
        DESTINATION share/${PROJECT_NAME}/param
)

# small_house.world is a symlink into the aws_robomaker_small_house_world checkout in src/third_party
install(DIRECTORY worlds/gazebo
        DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
