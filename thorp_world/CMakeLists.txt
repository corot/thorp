cmake_minimum_required(VERSION 3.8)
project(thorp_world)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS nodes/world_node.py scripts/check_world_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py)
  ament_add_pytest_test(test_conversions test/test_conversions.py)
endif()

ament_package()
