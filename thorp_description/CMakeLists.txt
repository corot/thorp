cmake_minimum_required(VERSION 3.10)
project(thorp_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()


#############
## Install ##
#############

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
