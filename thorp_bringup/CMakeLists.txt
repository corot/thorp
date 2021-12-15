cmake_minimum_required(VERSION 3.10)
project(thorp_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()


#############
## Install ##
#############

install(PROGRAMS scripts/fake_joint_pub.py
                 scripts/fake_servos_srv.py
                 scripts/gripper_cmd.py
                 scripts/keyboard_ctrl.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
