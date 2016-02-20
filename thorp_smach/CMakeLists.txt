cmake_minimum_required(VERSION 2.8.3)
project(thorp_smach)

catkin_python_setup()

catkin_package()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
