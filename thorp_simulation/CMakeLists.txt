cmake_minimum_required(VERSION 3.8)
project(thorp_simulation)

find_package(ament_cmake REQUIRED)

install(FILES launch/thorp_gazebo.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES param/gz_bridge.yaml
        DESTINATION share/${PROJECT_NAME}/param
)

install(FILES worlds/gazebo/empty.world worlds/gazebo/playground.world
        DESTINATION share/${PROJECT_NAME}/worlds/gazebo
)

install(DIRECTORY worlds/gazebo/models
        DESTINATION share/${PROJECT_NAME}/worlds/gazebo
)

ament_package()
