# Thorp action results typically contain an error code and an optional descriptive text
# The code can be either one of the codes described on moveit_msgs/MoveItErrorCodes:
#   int32 SUCCESS=1
#   int32 FAILURE=99999
#   int32 PLANNING_FAILED=-1
#   int32 INVALID_MOTION_PLAN=-2
#   int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
#   int32 CONTROL_FAILED=-4
#   int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
#   int32 TIMED_OUT=-6
#   int32 PREEMPTED=-7
#   int32 START_STATE_IN_COLLISION=-10
#   int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
#   int32 GOAL_IN_COLLISION=-12
#   int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
#   int32 GOAL_CONSTRAINTS_VIOLATED=-14
#   int32 INVALID_GROUP_NAME=-15
#   int32 INVALID_GOAL_CONSTRAINTS=-16
#   int32 INVALID_ROBOT_STATE=-17
#   int32 INVALID_LINK_NAME=-18
#   int32 INVALID_OBJECT_NAME=-19
#   int32 FRAME_TRANSFORM_FAILURE=-21
#   int32 COLLISION_CHECKING_UNAVAILABLE=-22
#   int32 ROBOT_STATE_STALE=-23
#   int32 SENSOR_INFO_STALE=-24
#   int32 NO_IK_SOLUTION=-31
# or, from a tree node wrapping a Nav2 action, that action's code,
# or a Thorp-specific error:
int32 SUCCESS=1
int32 OBJECT_NOT_FOUND=-200
int32 OBJECT_POSE_NOT_FOUND=-201
int32 OBJECT_SIZE_NOT_FOUND=-202
int32 INVALID_OBJECT_TYPE=-203
int32 TABLE_NOT_FOUND=-204
int32 INVALID_TARGET_POSE=-210
int32 INVALID_TARGET_TYPE=-211
int32 INVALID_NAMED_TARGET=-212
int32 INVALID_JOINT_STATE=-213
int32 JOINT_OUT_OF_BOUNDS=-214
int32 SERVER_NOT_AVAILABLE=-215
int32 OBJECT_NOT_ATTACHED=-220
int32 OBJECT_ATTACHED=-221
int32 NO_POSE_TIMEOUT=-222
int32 NOT_AT_TABLE=-230         # the robot isn't at the table the action needs
int32 OUT_OF_REACH=-231         # the object or the target pose is beyond the arm's reach from where the robot is
int32 OBJECT_NOT_ON_TABLE=-232  # the object is carried, or missing from where it was seen
int32 VIEW_BLOCKED=-233         # something close to the camera, as the arm when not stowed, blocks its view

int32  code
string text
