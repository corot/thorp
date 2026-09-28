cmake_minimum_required(VERSION 3.10)
project(thorp_apps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch param resources
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
