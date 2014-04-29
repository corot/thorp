cmake_minimum_required(VERSION 2.8.3)
project(thorp_boards)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  adc_driver
  gpio_driver
  arduino_interface
  roscpp
  sensor_msgs
  yocs_math_toolkit
)

find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thorp_boards
  CATKIN_DEPENDS adc_driver gpio_driver arduino_interface roscpp sensor_msgs yocs_math_toolkit
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(arduino_lib src/arduino/arduino.cpp)
add_dependencies(arduino_lib adc_driver gpio_driver arduino_interface)
target_link_libraries(arduino_lib ${catkin_LIBRARIES})

add_executable(arduino_node src/arduino/arduino_node.cpp)
add_dependencies(arduino_node arduino_lib sensor_msgs_gencpp)
target_link_libraries(arduino_node arduino_lib)


#############
## Install ##
#############

# TODO
