cmake_minimum_required(VERSION 3.8)
project(thorp_bringup)

find_package(ament_cmake REQUIRED)

# Parameters shared by the real robot and simulation; listed one by one, as the package is only partially ported
install(FILES param/cmd_vel_mux.yaml
        DESTINATION share/${PROJECT_NAME}/param
)
install(FILES param/kinect/depthimage_to_laserscan.yaml
        DESTINATION share/${PROJECT_NAME}/param/kinect
)
install(FILES param/xtion/pointcloud_to_laserscan.yaml
        DESTINATION share/${PROJECT_NAME}/param/xtion
)

install(FILES rviz/manipulation.rviz rviz/navigation.rviz
        DESTINATION share/${PROJECT_NAME}/rviz
)
install(DIRECTORY rviz/icons
        DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
