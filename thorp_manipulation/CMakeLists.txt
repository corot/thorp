cmake_minimum_required(VERSION 3.8)
project(thorp_manipulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS nodes/gripper_controller.py
                 nodes/fake_joint_pub.py
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES launch/includes/arm.launch.py
        DESTINATION share/${PROJECT_NAME}/launch/includes
)

install(FILES param/gripper_controller.yaml
        DESTINATION share/${PROJECT_NAME}/param
)

ament_package()
