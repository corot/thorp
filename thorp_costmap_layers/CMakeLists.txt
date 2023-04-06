cmake_minimum_required(VERSION 3.10)
project(thorp_costmap_layers)

# uncomment to enable OpenCV display of internally processed data (does not work on robot)
# add_definitions(-DDEBUG_DRAW_INTERNALS)
# add_definition(-DDEBUG_DRAW_HASH_INTERNALS)

# uncomment to enable GDB debugging
# set(CMAKE_BUILD_TYPE Debug)

if (NOT CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DNDEBUG)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  costmap_2d
  shape_msgs
  moveit_msgs
  geometry_msgs
  visualization_msgs
  dynamic_reconfigure
  thorp_toolkit
  thorp_msgs
)

generate_dynamic_reconfigure_options(
  cfg/SemanticLayer.cfg
)

find_package(Boost 1.58 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES semantic_layer
  CATKIN_DEPENDS
        tf2_ros
        roscpp
        costmap_2d
        moveit_msgs
        geometry_msgs
        visualization_msgs
        dynamic_reconfigure
        thorp_toolkit
        thorp_msgs
)

#######################################################################################################################
# semantic layer
#######################################################################################################################
add_library(semantic_layer
  src/semantic_layer.cpp
  src/base_interface.cpp
  src/srv_interface.cpp
  src/visualization.cpp
  src/spatial_hash.cpp
)

target_link_libraries(semantic_layer
  ${JSONCPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(semantic_layer ${PROJECT_NAME}_gencfg)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(semantic_layer ${catkin_EXPORTED_LIBS})
endif()

install(TARGETS semantic_layer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
