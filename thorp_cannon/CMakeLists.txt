cmake_minimum_required(VERSION 2.8.3)
project(thorp_cannon)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()


#############
## Install ##
#############

install(PROGRAMS nodes/cannon_ctrl.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

