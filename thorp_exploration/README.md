thorp_exploration
=================

Exploration planner (`nodes/exploration_planner.py`, node `exploration`), for the explore_house and object_gatherer
apps. On the map received on `map`, it offers these services:

- `~/segment_rooms` (`thorp_msgs/SegmentRooms`): segments the map into rooms, eroding the free space until regions split
  off, and growing them back.
- `~/plan_room_sequence` (`thorp_msgs/PlanRoomSequence`): orders the rooms to visit them all from a start pose,
  traveling the least.
- `~/plan_room_exploration` (`thorp_msgs/PlanRoomExploration`): plans the poses from where the camera sees all of a room,
  or of the whole map, in the order to visit them traveling the least.

It shows the rooms and the last poses planned, with the camera's field of view on each, as markers on `~/rooms` and
`~/viewpoints`. `launch/exploration.launch.py` runs it for the Kinect or Xtion camera; `param/exploration.yaml` has the
fields of view and the planning parameters. On Noetic, `ipa_coverage_planning` did this job.
