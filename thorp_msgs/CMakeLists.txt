cmake_minimum_required(VERSION 2.8.3)
project(thorp_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES KeyboardInput.msg
                 )

add_action_files(DIRECTORY action
                     FILES DetectObjects.action
                           MoveToTarget.action
                           PickupObject.action
                           PlaceObject.action
                           InteractiveManip.action
                           UserCommand.action
                )

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs)
