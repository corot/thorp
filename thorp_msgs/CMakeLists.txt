cmake_minimum_required(VERSION 3.8)
project(thorp_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BTNodeStatus.msg
  msg/ObjectToPickup.msg
  msg/PickupLocation.msg
  msg/Room.msg
  msg/ThorpError.msg
  srv/CannonCommand.srv
  srv/ClearPlanningScene.srv
  srv/PlanRoomExploration.srv
  srv/PlanRoomSequence.srv
  srv/SegmentRooms.srv
  action/DetectObjects.action
  action/DetectTables.action
  action/DragAndDrop.action
  action/FollowPose.action
  action/PickupObject.action
  action/PlaceObject.action
  action/RunSubtree.action
  DEPENDENCIES builtin_interfaces geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
