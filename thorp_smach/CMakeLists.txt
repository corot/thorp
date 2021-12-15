cmake_minimum_required(VERSION 3.10)
project(thorp_smach)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
