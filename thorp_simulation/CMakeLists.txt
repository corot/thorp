cmake_minimum_required(VERSION 2.8.3)
project(thorp_simulation)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED roscpp gazebo_msgs geometry_msgs thorp_msgs thorp_toolkit gazebo_grasp_plugin)
catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_executable(gazebo_ground_truth src/gazebo_ground_truth.cpp)
target_link_libraries(gazebo_ground_truth ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(gazebo_camera_control src/gazebo_camera_control.cpp)
target_link_libraries(gazebo_camera_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(gazebo_topics_relay src/gazebo_topics_relay.cpp)
add_dependencies(gazebo_topics_relay thorp_msgs_generate_messages_cpp)
target_link_libraries(gazebo_topics_relay ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} gazebo_grasp_msgs)

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
