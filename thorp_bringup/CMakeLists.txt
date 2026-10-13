cmake_minimum_required(VERSION 3.8)
project(thorp_bringup)

find_package(ament_cmake REQUIRED)

# Parameters shared by the real robot and simulation; listed one by one, as the package is only partially ported
install(FILES config/twist_mux.yaml
        DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY config/named_configs
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES config/kinect/depthimage_to_laserscan.yaml
        DESTINATION share/${PROJECT_NAME}/config/kinect
)
install(FILES config/xtion/pointcloud_to_laserscan.yaml
        DESTINATION share/${PROJECT_NAME}/config/xtion
)

install(PROGRAMS nodes/named_configs.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
