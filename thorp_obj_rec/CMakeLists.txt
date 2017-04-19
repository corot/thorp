cmake_minimum_required(VERSION 2.8.3)
project(thorp_obj_rec)

find_package(catkin REQUIRED roscpp actionlib thorp_toolkit moveit_ros_planning_interface
                             thorp_msgs object_recognition_msgs geometry_msgs moveit_msgs shape_msgs
                             image_geometry image_transport cv_bridge mag_common_cpp_libs)
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(DEPENDS roscpp actionlib thorp_toolkit moveit_ros_planning_interface
                       thorp_msgs object_recognition_msgs geometry_msgs moveit_msgs shape_msgs
                       image_geometry image_transport cv_bridge mag_common_cpp_libs)

# nodes

add_executable(object_detection_server src/object_detection_action_server.cpp)
target_link_libraries(object_detection_server ${catkin_LIBRARIES})
add_dependencies(object_detection_server thorp_msgs_generate_messages_cpp)


# install

install(TARGETS object_detection_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
