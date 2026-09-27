cmake_minimum_required(VERSION 3.8)
project(thorp_toolkit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/common.cpp
  src/geometry.cpp
  src/progress_tracker.cpp
  src/simulation.cpp
  src/tf2.cpp
  src/visualization.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  angles::angles
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${rviz_2d_overlay_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
  Boost::boost
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES include/thorp_toolkit/common.hpp
              include/thorp_toolkit/geometry.hpp
              include/thorp_toolkit/math.hpp
              include/thorp_toolkit/parameters.hpp
              include/thorp_toolkit/progress_tracker.hpp
              include/thorp_toolkit/simulation.hpp
              include/thorp_toolkit/tf2.hpp
              include/thorp_toolkit/visualization.hpp
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

# Python modules; listed one by one, as the package is only partially ported
_ament_cmake_python_get_python_install_dir()
install(FILES python/thorp_toolkit/__init__.py
              python/thorp_toolkit/common.py
              python/thorp_toolkit/decorators.py
              python/thorp_toolkit/geometry.py
              python/thorp_toolkit/progress_tracker.py
              python/thorp_toolkit/singleton.py
              python/thorp_toolkit/spatial_hash.py
              python/thorp_toolkit/tachometer.py
              python/thorp_toolkit/transform.py
              python/thorp_toolkit/visualization.py
        DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_progress_tracker test/test_progress_tracker.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp angles std_msgs geometry_msgs visualization_msgs rviz_2d_overlay_msgs
                          tf2 tf2_ros tf2_eigen tf2_geometry_msgs Eigen3 Boost)
ament_package()
