# Pick location; the distance is from robot location at picking planning time
string name
float32 distance
thorp_msgs/ObjectToPick[] objects  # reachable objects from this location
geometry_msgs/PoseStamped arm_pose
geometry_msgs/PoseStamped approach_pose
geometry_msgs/PoseStamped picking_pose
geometry_msgs/PoseStamped detach_pose