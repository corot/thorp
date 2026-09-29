# Object to add to / remove from the semantic layer using the update_objects service

uint8 ADD    = 1
uint8 REMOVE = 2

uint8 operation

string name
string type
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 dimensions
