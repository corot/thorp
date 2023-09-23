cmake_minimum_required(VERSION 3.10)
project(thorp_apps)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
