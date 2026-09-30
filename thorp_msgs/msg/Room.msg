# A room of the map, as the exploration planner segments it
uint32 id                   # from 1
float32 area                # free space, in square meters
geometry_msgs/Point center  # the room's free point farthest from obstacles, on the map frame
