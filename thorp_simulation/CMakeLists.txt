cmake_minimum_required(VERSION 2.8.3)
project(thorp_simulation)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp gazebo_msgs geometry_msgs thorp_toolkit)
catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(gazebo_camera_control src/gazebo_camera_control.cpp)
target_link_libraries(gazebo_camera_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
