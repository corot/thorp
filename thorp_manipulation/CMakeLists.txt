cmake_minimum_required(VERSION 3.8)
project(thorp_manipulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS nodes/fake_joint_pub.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/includes/arm.launch.py
        DESTINATION share/${PROJECT_NAME}/launch/includes
)

ament_package()
