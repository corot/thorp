# thorp_world

What Thorp knows about the world: tables and the objects on them, in the `map` frame, with the relations between them.
It is the long-term memory the executive and the LLM agent share, so that skills can take entity ids instead of values
passed from one call to the next, and the agent can reason on what is where without being told how to reach it.

## Model

- Entities: tables and objects, each with an id (`table 1`, `cube 3`: its type and an index, as detection names them),
  type, color, size and, while on a table, a pose in `map`. Tables have x along their longest side, and yaw within
  (-π/2, π/2], as a table is the same rotated half a turn.
- Support: what an object rests on, a table id, `gripper` or `tray` (with a slot). Carried objects have no pose, as
  the robot moves them.
- Provenance: `observed` (a detection saw it so) or `believed` (a skill left it so), with the stamp of when.
- Status: `ok`, `disturbed` (a skill touched it without the intended result) or `missing` (not seen where it was, from
  at its table).
- Inspection: when the robot last looked each table over, from a viewing pose (`validate_table`, so `inspect_table`
  too), telling the tables visited from those only glimpsed.
- The robot: only what TF can't tell, the table it's docked at.
- Relations, derived on query: an object's support; whether it's within reach of the arm's base; the table the robot
  is at: the one it docked at if it's still close to it, or else the closest one that close.

Observations are matched against what is known: an object is the known object closest to it, within a tolerance,
whatever type detection gives it this time, as it sometimes misidentifies one; the object keeps the type it was first
seen as. A table is the known table its center falls on. A table's pose and size are replaced only by views at least
about as large, as partial views give smaller tables. Objects known on a table become missing only when a detection
from at that table doesn't see them, and only where the observed surface covers them: from further away, as from
`inspect_table`'s viewing pose, detection misses objects that are there.

## Node

`world_node.py`, launched by `world.launch.py` with its file as `thorp_world_file`, serves under its name (`world`):

| Interface | Type | Use |
|---|---|---|
| `world/observe` | `thorp_msgs/ObserveWorld` | Detections report tables, or a surface with the objects on it, as the collision objects they produce, on any frame; the world transforms them to `map` at their stamp, matches them and returns their ids. |
| `world/update` | `thorp_msgs/UpdateWorld` | Skills report an effect: an object picked, placed, stowed, disturbed or lost from the gripper; the robot docked or undocked; a table inspected. Unknown ids are refused. |
| `world/query` | `thorp_msgs/QueryWorld` | Entities filtered by ids, kind, support or status, with each one's reachability, as the manipulation server measures it: how far its top is from the arm's shoulder, and how much above or below it (`arm_frame`, `max_arm_reach`, `max_arm_height`) and the table the robot is at (`at_table_distance`). |
| `world/reach` | `thorp_msgs/QueryReach` | Which objects on a table, not missing, the arm would reach with the robot at each of some poses: the same reach as `world/query`'s, less a margin for poses reached with some tolerance. The pickup planner groups objects into pickup locations by it. |
| `world/save`, `world/load` | `std_srvs/Trigger` | To and from `world_file`, also loaded on start if it exists. The parameter can change at any time: the capability tests point it at the world they start from. |
| `world/state` | `thorp_msgs/WorldState` | Everything known, latched, whenever it changes. |
| `world/markers` | `visualization_msgs/MarkerArray` | Tables and objects on a table, labeled with their ids: gray if missing or disturbed, faded if only believed. |

`scripts/check_world_node.py` exercises a running node against static transforms; its docstring tells how. Run it on
a ROS domain nothing else uses: static transforms published beside a simulation stay in its nodes' buffers.

## Interactions

| Node | With thorp_world |
|---|---|
| `object_detection` | Reports each detection to `world/observe`, in its own frame; the world transforms it to `map`, matches it and returns the ids, that detection gives the planning scene's table and objects. The agent, the planning scene and the pick and place servers so use the same names. |
| MoveIt planning scene | Stays robot-relative, the short-lived working copy of the arm, refreshed by detecting at the table. The world never writes it. |
| Tray | The planning scene stays the truth of which slots are free, as the tray moves with the robot; the world mirrors what each slot holds, as place skills report. |
| Behavior trees | `GetTable`, `GetCurrentTable`, `ObjectPickable` and `TableVisited` read it, and `PlanPickupLocations` its reach; `UpdateWorld` reports skills' effects, on success and failure. The table and manipulation skills take ids: `reach_table`, `inspect_table` and `cleanup_table` an optional `table_id`, `go_to_table` a `table_id` and `pickup_object` an object's; they fail with `TABLE_NOT_FOUND`, `OBJECT_NOT_FOUND`, `OBJECT_NOT_ON_TABLE`, `OBJECT_ATTACHED`, `NOT_AT_TABLE` or `OUT_OF_REACH` when the world disagrees. |
| Semantic costmap layer | Apart for now: the table nodes mark tables as obstacles and clear a table's access through its services. Reading `world/state` instead is for later: docking needs the access cleared before the world learns of the dock, and the side to clear is the pickup pose's, which the world doesn't keep. |
| LLM agent | `thorp_agent`: skills take ids; each request reaches the model after a summary of the world (the robot's table, gripper and tray, and each table with its objects, their colors, status and reach); each capability result lists what the call changed (`world_changes`: an object picked, stowed or found, a table inspected, the robot at another table, what the arm reaches now); and a `query_world` tool gives the world in full. |
| RViz | `world/markers`, labeled with the ids; faded if only believed, gray if missing or disturbed. |
| Simulation | A checker compares `world/state` with Gazebo's model poses, telling perception errors from reasoning errors. |

## Steps

| # | Step | Status |
|---|------|--------|
| 1 | Core in plain Python: entities, matching, relations, provenance, persistence; with tests | done |
| 2 | ROS 2 node: `thorp_msgs` interfaces (entity, state, observe, update, query, save and load), RViz markers | done |
| 3 | Perception reports to it, and names objects with its ids; the planning scene's table keeps the name `table` until the skills take table ids | done |
| 4a | Tree nodes: `GetTable`, `GetCurrentTable`, `ObjectPickable` (failing with `OBJECT_NOT_FOUND`, `OBJECT_NOT_ON_TABLE`, `NOT_AT_TABLE` or `OUT_OF_REACH`) and `UpdateWorld`; `test_world_nodes.py` runs them against `test/world.yaml` | done |
| 4b | Table and manipulation skills take ids, check them on the world and report their effects; the planning scene's table takes its world id | done |
| 4c | `capabilities.yaml` drops `states`, `needs`, `invalidates` and `from`, and gains `errors`; tests start from a preloaded world instead of `--given` | done |
| 4d | `PlanPickupLocations` asks the world for reach, and `TableVisited` whether a table was inspected; the semantic layer stays apart for now | done |
| 5 | Agent on ROS 2 (ROSA): world tools, world summary, world changes in skill results | done; untested with a real model |

`thorp_world` builds with the rest of the workspace. Its own tests are the core's and the conversions'; the tree nodes'
are `thorp_bt_cpp`'s `test/test_world_nodes.py`, and the agent's `thorp_agent`'s `test/test_knowledge.py`:

```bash
cd ~/colcon_ws/thorp
source install/setup.bash
colcon test --packages-select thorp_world && colcon test-result --verbose
```
