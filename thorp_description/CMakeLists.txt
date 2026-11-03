cmake_minimum_required(VERSION 3.10)
project(thorp_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
