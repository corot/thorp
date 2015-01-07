cmake_minimum_required(VERSION 2.8.3)
project(thorp_smach)

find_package(catkin REQUIRED COMPONENTS actionlib
                                        actionlib_msgs
                                        control_msgs
                                        geometry_msgs
                                        manipulation_msgs
                                        trajectory_msgs
                                        moveit_msgs
                                        sensor_msgs
                                        shape_msgs
                                        smach
                                        smach_ros
                                        std_msgs
                                        std_srvs
                                        rospy
                                        tf)

catkin_python_setup()

catkin_package()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
