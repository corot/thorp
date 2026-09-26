cmake_minimum_required(VERSION 3.8)
project(thorp_cannon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

# Gazebo Harmonic system simulating cannon firing
add_library(thorp_cannon_system SHARED src/cannon_system.cpp)
target_link_libraries(thorp_cannon_system
  gz-sim::gz-sim
  gz-plugin::register
  gz-msgs::gz-msgs
  gz-transport::gz-transport
)

install(TARGETS thorp_cannon_system
        LIBRARY DESTINATION lib
)

install(PROGRAMS nodes/cannon_ctrl.py
                 scripts/cannon_cmd.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
