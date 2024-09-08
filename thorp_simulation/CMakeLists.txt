cmake_minimum_required(VERSION 3.10)
project(thorp_simulation)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp gazebo_msgs geometry_msgs thorp_toolkit)

catkin_package(CATKIN_DEPENDS roscpp gazebo_msgs geometry_msgs thorp_toolkit)

include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(gazebo_camera_control src/gazebo_camera_control.cpp)
target_link_libraries(gazebo_camera_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_camera_control_plugin src/gazebo_camera_control_plugin.cpp)
target_link_libraries(gazebo_camera_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ground_truth gazebo_camera_control gazebo_camera_control_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/gazebo_link_state.py
                               scripts/spawn_gazebo_models.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS nodes/cats_controller.py
                               nodes/model_markers.py
                               nodes/movie_director.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
