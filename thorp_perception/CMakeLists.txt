cmake_minimum_required(VERSION 2.8.3)
project(thorp_perception)

find_package(catkin REQUIRED tf roscpp pcl_ros actionlib
                             moveit_ros_planning_interface
                             thorp_msgs thorp_toolkit
                             object_recognition_msgs
                             rail_manipulation_msgs rail_mesh_icp
                             geometry_msgs moveit_msgs shape_msgs
                             image_geometry image_transport cv_bridge)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(INCLUDE_DIRS include
               DEPENDS tf roscpp pcl_ros actionlib
                       moveit_ros_planning_interface
                       thorp_msgs thorp_toolkit
                       object_recognition_msgs
                       rail_manipulation_msgs rail_mesh_icp
                       geometry_msgs moveit_msgs shape_msgs
                       image_geometry image_transport cv_bridge)

# nodes

add_executable(ork_object_detection_server src/ork_object_detection_server.cpp)
target_link_libraries(ork_object_detection_server ${catkin_LIBRARIES})
add_dependencies(ork_object_detection_server thorp_msgs_generate_messages_cpp)

add_executable(rail_object_detection_server src/rail_object_detection_server.cpp)
target_link_libraries(rail_object_detection_server ${catkin_LIBRARIES})
add_dependencies(rail_object_detection_server thorp_msgs_generate_messages_cpp rail_mesh_icp_generate_messages_cpp)


# install

install(TARGETS ork_object_detection_server rail_object_detection_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
