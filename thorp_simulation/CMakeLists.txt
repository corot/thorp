cmake_minimum_required(VERSION 2.8.3)
project(thorp_simulation)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp)
catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_executable(gz_camera_control src/gz_camera_control.cpp)
target_link_libraries(gz_camera_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
