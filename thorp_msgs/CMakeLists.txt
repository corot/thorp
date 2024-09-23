cmake_minimum_required(VERSION 3.10)
project(thorp_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES BTNodeStatus.msg
                            KeyboardInput.msg
                            ObjectToPickup.msg
                            PickupPlan.msg
                            PickupLocation.msg
                            ThorpError.msg
                 )

add_service_files(DIRECTORY srv
                     FILES CannonCommand.srv
                           ClearPlanningScene.srv
                           ConnectWaypoints.srv
                           TrayAddObject.srv
                           TrayCapacity.srv
                           TrayNextPose.srv
                )

add_action_files(DIRECTORY action
                     FILES DetectObjects.action
                           DragAndDrop.action
                           FollowPose.action
                           MakePickupPlan.action
                           MoveToTarget.action
                           PickupObject.action
                           PlaceObject.action
                           UserCommand.action
                )

generate_messages(DEPENDENCIES std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs moveit_msgs geometry_msgs actionlib_msgs)
