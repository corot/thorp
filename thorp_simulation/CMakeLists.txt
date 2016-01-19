cmake_minimum_required(VERSION 2.8.3)
project(thorp_simulation)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/tf_connector.py    
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
