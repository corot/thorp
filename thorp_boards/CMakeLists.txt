cmake_minimum_required(VERSION 2.8.3)
project(thorp_boards)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thorp_boards
  CATKIN_DEPENDS roscpp serial sensor_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(arduino_lib src/arduino/arduino.cpp)
add_dependencies(arduino_lib sensor_msgs_generate_messages_cpp)
target_link_libraries(arduino_lib ${catkin_LIBRARIES})

add_executable(arduino_node src/arduino/arduino_node.cpp)
add_dependencies(arduino_node arduino_lib sensor_msgs_generate_messages_cpp)
target_link_libraries(arduino_node arduino_lib)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS arduino_lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS arduino_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
