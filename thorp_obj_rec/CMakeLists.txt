cmake_minimum_required(VERSION 2.8.3)
project(thorp_obj_rec)

find_package(catkin REQUIRED roscpp actionlib thorp_toolkit yocs_math_toolkit moveit_ros_planning_interface
                             thorp_msgs object_recognition_msgs geometry_msgs moveit_msgs shape_msgs)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(DEPENDS roscpp actionlib thorp_toolkit yocs_math_toolkit moveit_ros_planning_interface
                       thorp_msgs object_recognition_msgs geometry_msgs moveit_msgs shape_msgs)

# nodes

add_executable(object_detection_server src/object_detection_action_server.cpp)
target_link_libraries(object_detection_server ${catkin_LIBRARIES})
add_dependencies(object_detection_server thorp_msgs_gencpp)


# install

install(TARGETS object_detection_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
