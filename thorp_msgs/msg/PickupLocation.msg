# Pickup location; the distance is from robot location at pickup planning time
string name
float32 distance
thorp_msgs/ObjectToPickup[] objects  # reachable objects from this location
geometry_msgs/PoseStamped arm_pose
geometry_msgs/PoseStamped approach_pose
geometry_msgs/PoseStamped pickup_pose
geometry_msgs/PoseStamped detach_pose
