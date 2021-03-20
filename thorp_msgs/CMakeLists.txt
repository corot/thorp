cmake_minimum_required(VERSION 2.8.3)
project(thorp_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES KeyboardInput.msg
                            ThorpError.msg
                 )

add_service_files(DIRECTORY srv
                     FILES CannonCmd.srv
                           ConnectWaypoints.srv
                )

add_action_files(DIRECTORY action
                     FILES DetectObjects.action
                           MoveToTarget.action
                           PickupObject.action
                           PlaceObject.action
                           DragAndDrop.action
                           UserCommand.action
                           FollowPose.action
                )

generate_messages(DEPENDENCIES std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs)
