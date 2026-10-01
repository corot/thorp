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
- Status: `ok`, `disturbed` (a skill touched it without the intended result) or `missing` (not seen where it was, on a
  table in view).
- The robot: only what TF can't tell, the table it's docked at.
- Relations, derived on query: an object's support; whether it's within reach of the arm's base; the table the robot
  is docked at, if it's still at it.

Observations are matched against what is known: an object is the known object of its type closest to it, within a
tolerance; a table, the known table its center falls on. A table's pose and size are replaced only by views at least
about as large, as partial views give smaller tables. Objects known on a table become missing only where the observed
surface covers them.

## Node

`world_node.py`, launched by `world.launch.py`, serves under its name (`world`):

| Interface | Type | Use |
|---|---|---|
| `world/observe` | `thorp_msgs/ObserveWorld` | Detections report tables, or a surface with the objects on it, as the collision objects they produce, on any frame; the world transforms them to `map` at their stamp, matches them and returns their ids. |
| `world/update` | `thorp_msgs/UpdateWorld` | Skills report an effect: an object picked, placed, stowed or disturbed; the robot docked or undocked. Unknown ids are refused. |
| `world/query` | `thorp_msgs/QueryWorld` | Entities filtered by ids, kind, support or status, with each one's reachability from the arm's base (`arm_frame`, `max_arm_reach`) and the table the robot is still docked at (`dock_distance`). |
| `world/save`, `world/load` | `std_srvs/Trigger` | To and from `world_file`, also loaded on start if it exists. |
| `world/state` | `thorp_msgs/WorldState` | Everything known, latched, whenever it changes. |
| `world/markers` | `visualization_msgs/MarkerArray` | Tables and objects on a table, labeled with their ids: gray if missing or disturbed, faded if only believed. |

`scripts/check_world_node.py` exercises a running node against static transforms; its docstring tells how. Run it on
a ROS domain nothing else uses: static transforms published beside a simulation stay in its nodes' buffers.

## Interactions

| Node | With thorp_world |
|---|---|
| `object_detection` | Reports each detection to `world/observe`, in its own frame; the world transforms it to `map`, matches it and returns the ids, that detection gives the planning scene objects. The agent, the planning scene and the pick and place servers so use the same names. |
| MoveIt planning scene | Stays robot-relative, the short-lived working copy of the arm, refreshed by detecting at the table. The world never writes it. |
| Tray | The planning scene stays the truth of which slots are free, as the tray moves with the robot; the world mirrors what each slot holds, as place skills report. |
| Behavior trees | `GetEntity` and `QueryWorld` nodes read it; `UpdateWorld` reports skills' effects, on success and failure. |
| Semantic costmap layer | Subscribes to `world/state`: tables as obstacles, and the approach area of the table the robot is docked at cleared. |
| LLM agent | Skills take ids; a `query_world` tool, a world summary with each request and the world changes with each skill result. |
| RViz | `world/markers`, labeled with the ids; faded if only believed, gray if missing or disturbed. |
| Simulation | A checker compares `world/state` with Gazebo's model poses, telling perception errors from reasoning errors. |

## Steps

| # | Step | Status |
|---|------|--------|
| 1 | Core in plain Python: entities, matching, relations, provenance, persistence; with tests | done |
| 2 | ROS 2 node: `thorp_msgs` interfaces (entity, state, observe, update, query, save and load), RViz markers | done |
| 3 | Perception reports to it, and names objects with its ids; the planning scene's table keeps the name `table` until the skills take table ids | done |
| 4a | Tree nodes: `GetTable`, `GetDockedTable`, `ObjectPickable` (failing with `OBJECT_NOT_FOUND`, `OBJECT_NOT_ON_TABLE`, `NOT_AT_TABLE` or `OUT_OF_REACH`) and `UpdateWorld`; `test_world_nodes.py` runs them against `test/world.yaml` | done |
| 4b | Table and manipulation skills take ids, check them on the world and report their effects; the planning scene's table takes its world id | |
| 4c | `capabilities.yaml` drops `states`, `needs`, `invalidates` and `from`, and gains `errors`; tests start from a preloaded world instead of `--given` | |
| 4d | `PlanPickupLocations`, the semantic layer and `TableVisited` read the world | |
| 5 | Agent on ROS 2 (ROSA): world tools, world summary, world changes in skill results | |

The work happens on the `thorp_world` branch, in a worktree with its own colcon workspace over the main one's install:

```bash
cd ~/colcon_ws/thorp_world
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/thorp/install/setup.bash
colcon build --symlink-install --packages-select thorp_world
colcon test --packages-select thorp_world && colcon test-result --verbose
```
