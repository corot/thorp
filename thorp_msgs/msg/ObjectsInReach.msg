# The objects on a table the arm reaches with the robot at robot_pose, closest first
geometry_msgs/PoseStamped robot_pose  # on the map frame
geometry_msgs/PoseStamped arm_pose    # the arm's shoulder, that reach is measured from, with the robot there
ObjectToPickup[] objects
