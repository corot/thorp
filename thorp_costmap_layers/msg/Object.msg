# Object to add to / remove from the semantic layer using the update_objects service

byte ADD    = 1
byte REMOVE = 2

byte operation

string name
string type
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 dimensions
