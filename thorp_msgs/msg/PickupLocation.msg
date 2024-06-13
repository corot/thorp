# Pickup location; the distance is from robot location at pickup planning time
string name
float32 distance
thorp_msgs/ObjectToPickup[] objects      # Reachable objects from this location
geometry_msgs/PoseStamped arm_pose       # Pose of the arm base at this pickup location
geometry_msgs/PoseStamped approach_pose  # Robot pose before attaching to the target
geometry_msgs/PoseStamped pickup_pose    # Robot pose while picking at this location
geometry_msgs/PoseStamped detach_pose    # Robot pose after detaching from the target
