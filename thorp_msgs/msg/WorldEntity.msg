# A table or an object, as thorp_world knows it

string TABLE="table"
string OBJECT="object"
string GRIPPER="gripper"
string TRAY="tray"
string OBSERVED="observed"
string BELIEVED="believed"
string OK="ok"
string DISTURBED="disturbed"
string MISSING="missing"

string id                       # its type and an index, e.g. "cube 3"
string kind                     # TABLE or OBJECT
string type                     # "table", or the detection template, e.g. "cube"
string color                    # a color name, as detections report them
geometry_msgs/Vector3 size      # tables have x along their longest side
bool has_pose                   # false while carried, as the robot moves it
geometry_msgs/PoseStamped pose  # in the map frame
string support                  # objects: a table id, GRIPPER or TRAY
int32 slot                      # on the tray; -1 otherwise
string provenance               # OBSERVED (a detection saw it so) or BELIEVED (a skill left it so)
builtin_interfaces/Time stamp   # when last observed, or last changed by a skill
string status                   # OK, DISTURBED (touched without the intended result) or MISSING (not seen where it was)
builtin_interfaces/Time inspected  # tables: when the robot last looked one over; zero if never
