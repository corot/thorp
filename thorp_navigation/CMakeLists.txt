cmake_minimum_required(VERSION 3.8)
project(thorp_navigation)

find_package(ament_cmake REQUIRED)

# Listed one by one, as the package is only partially ported
install(PROGRAMS nodes/show_velocity.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/navigation.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES param/nav2.yaml
        DESTINATION share/${PROJECT_NAME}/param
)

install(DIRECTORY behavior_trees
        DESTINATION share/${PROJECT_NAME}
)

# small_house map files are symlinks into the aws_robomaker_small_house_world checkout in src/third_party
install(DIRECTORY maps
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
