# Thorp on ROS 2 Jazzy

The `jazzy` branch is the ROS 2 port of Thorp. The `noetic` branch keeps the ROS 1 version.

## Rules

- Target: ROS 2 Jazzy and Gazebo Harmonic, native ROS 2 only. No `ros1_bridge`, no mixed ROS 1 / ROS 2 runtime.
- Migrate in small blocks. Each block builds and runs on Jazzy on its own.
- Simulation first. Real hardware (Kobuki, arm servos, boards) is out of scope for now, but frame names, joint names
  and topics stay compatible with the real robot, so it can come back later.
- A package that is not migrated yet has a `COLCON_IGNORE` file; colcon and rosdep skip it. Migrating a package means
  porting it completely and deleting that file. Its ROS 1 code stays in place until then, for reference.
- A package whose parts depend on later blocks is migrated in stages: its `COLCON_IGNORE` goes with the first stage,
  it builds and installs only the ported parts, and the unported ROS 1 files stay in place, uninstalled, listed below
  with their target block.
- Prefer Jazzy solutions over Noetic's design: when a Jazzy binary package does the job of a Thorp node or plugin, use
  it instead of porting or writing code. Interfaces (frames, joints, topics) still stay compatible.
- Dependencies: Jazzy binary packages first. A dependency with no Jazzy release goes into `thorp-jazzy.repos`
  (created when the first one is needed), pinned to a commit SHA. If it needs patches, use a fork and note the
  upstream URL and the reason next to its entry.
- Robot description data (xacro and meshes) from unreleased ROS 1 packages is copied into `thorp_description`, with
  a README recording its source and license.
- Style: "Thorp" in prose, `thorp` in code. Keep the existing Python style; don't run black.

## Workspace

```text
~/colcon_ws/thorp/
  src/thorp/          # this repository, jazzy branch
  src/third_party/    # source dependencies from thorp-jazzy.repos
```

Build:

```bash
cd ~/colcon_ws/thorp
source /opt/ros/jazzy/setup.bash
vcs import src/third_party < src/thorp/thorp-jazzy.repos
rosdep install --from-paths src --ignore-src -r -y --skip-keys "python3-torchvision-pip python3-ultralytics-pip"
colcon build --symlink-install
source install/setup.bash
```

`yolo_ros` needs `uv` (`pipx install uv`): its build creates a Python virtual environment in its source tree, with
the PyTorch and Ultralytics versions it pins, and points its nodes at it, so no other node sees them. Hence rosdep skips
the pip keys it declares for them.

## Blocks

| # | Block | Status |
|---|-------|--------|
| 1 | Workspace bootstrap: branch, `COLCON_IGNORE` on every ROS 1 package, this document | done |
| 2 | `thorp_description`: robot model identical to Noetic's, RViz viewer | done |
| 3 | Gazebo Harmonic: spawn Thorp, diff drive, joint states, Kinect, Xtion, sonars and IR sensors | done |
| 4 | Arm in simulation: `ros2_control`, trajectory and gripper controllers | done |
| 5 | `thorp_msgs`, `thorp_toolkit`; `thorp_cannon`, with a Gazebo Harmonic firing system | done |
| 6a | Nav2 core: map, localization (AMCL, static, Gazebo ground truth), depth cameras to laser scans, costmaps (static, voxel, sonar and IR range layers, inflation), planner, MPPI controller, behaviors, velocity smoother, velocity commands multiplexer, RViz goals | done |
| 6b | Thorp navigation nodes: pose follower, waypoints path, velocity display, robot pose saving | done |
| 6c | Semantic costmap layer | done in 9b |
| 6d | Bumpers and cliff sensors, on simulation and costmaps | deferred to real robot |
| 6e | Coverage planning | done in 9c |
| 7a | MoveIt 2 configuration: move group, controllers, octomap from the Xtion, RViz; pick_ik replaces the IKFast plugin | done |
| 7b | Pickup and place object action servers on MoveIt Task Constructor | done |
| 7c | Grasping in simulation and object spawning | done |
| 8 | Perception: tables and tabletop objects detection | done |
| 9a | Behavior trees framework on BehaviorTree.CPP 4 (runner, `bt_server`, JSON blackboard), manipulation and perception capabilities; `object_manip` and `pickup_objects` apps | done |
| 9b | Navigation capabilities on Nav2, semantic costmap layer, pickup planner; `patrol_2_points` and `cleanup_table` apps | done |
| 9c | Exploration: coverage planning and room segmentation; `explore_house` and `object_gatherer` apps | done |
| 9d | Cat hunter: object detection replacing COB, cats models and controller | next |
| 9e | LLM agent (`thorp_agent`) | |

Block 3 onwards will be refined as we get there.

Navigation uses Nav2 only; Move Base Flex is dropped, with whatever depends on it (`thorp_mbf_plugins`, MBF actions
and plugins configuration in `thorp_navigation`). The executive will use Nav2's own interfaces (`navigate_to_pose`,
`compute_path_to_pose`, `follow_path`, behaviors and the costmaps' `get_cost` service). MPPI replaces TEB, which has no
Jazzy release, and Nav2's standard behaviors replace `SlowEscapeRecovery` until the executive shows whether they are
enough.

The semantic costmap layer (`thorp_costmap_layers`) is only used by the executive, to mark tables as obstacles in both
costmaps (the scans can't see their eaves) and to clear an approach area in front of them in the local costmap. It's
a Nav2 layer plugin: Nav2 on Jazzy can mark regions with its keepout filter, fed with a mask of rectangles by a small
node, but can't clear regions persistently (only once, with `clear_around_pose`).

Block 9b moves the executive's navigation to Nav2's actions, using `nav2_behavior_tree`'s nodes where they fit:
`navigate_to_pose` for going to a pose, with one goal tolerance for all goals; `navigate_through_poses` for following
waypoints, its remaining poses dropping the passed ones; `follow_path` for docking at a table, straight to the pickup
pose; and the behaviors `spin` for turning to face a pose and `backup` for leaving a table. Noetic's named
configurations, that reconfigured TEB at runtime, become plugins chosen per goal: a precise controller and goal checker,
for docking at tables. The trees drop the bumper and safety controller nodes until the real robot, and the RAIL markers
clearing. `patrol_2_points` runs on the playground world, instead of Stage's maze, and a new `cleanup_table` app picks
the objects around the playground table.

Bumpers and cliff sensors are deferred to the real robot: it needs `kobuki_ros` from source anyway (the Kobuki driver,
`kobuki_bumper2pc` and `kobuki_safety_controller` have no Jazzy release), and simulated bumper and cliff events, from
Gazebo contact sensors and downward rays, can then match the real driver's topics.

The executive is ported from the `bt_server` branch, the latest: its behavior trees cover all the apps but
`stack_all_cubes`, and `bt_server` and the LLM agent build on them. Only the behavior trees are ported, on
BehaviorTree.CPP 4, as Nav2's, so the trees can use `nav2_behavior_tree`'s Nav2 nodes and ROS action and service node
templates instead of Thorp's ROS 1 ones. SMACH state machines are dropped, although `executive_smach` has a Jazzy
release; `stack_all_cubes` becomes a behavior tree.

MoveIt 2 has no pick and place capability (`moveit_msgs` keeps the `Pickup` and `Place` actions, but nothing serves
them), so the pickup and place object servers build MoveIt Task Constructor tasks instead, keeping their `thorp_msgs`
actions, now with the stage being executed as feedback. The rest of Noetic's manipulation servers go to the executive,
in Block 9: `move_to_target` (the behavior trees only use named targets, that `move_group` takes directly), the house
keeping services (clear gripper, force resting, object attached and gripper busy: planning scene queries and controller
commands), the tray manager (slot poses, now computed from the planning scene) and the drag and drop demo. So does the
pickup planner, that groups objects on a table into pickup locations: app-level planning, with no MoveIt use.

Coverage planning, for the exploration apps, is Thorp's own exploration planner (`thorp_exploration`). On Noetic it
was `ipa_coverage_planning` (room segmentation, room sequence planning and room exploration, from Thorp's fork) and
`full_coverage_path_planner`'s Spiral-STC planner, as an MBF global planner. No option is a Jazzy release:
`ipa_coverage_planning` has no ROS 2 port, and its dependencies include `opengm` and `cob_map_accessibility_analysis`,
not released for Jazzy either; `full_coverage_path_planner`'s `ros2` branch is an unfinished migration, untouched since
2023; and `opennav_coverage` (Nav2 coverage server, source only; its `jazzy-v2` branch takes Jazzy's Fields2Cover 2.0)
plans swaths over field polygons, without room segmentation nor sequencing.

## Package status

| Package | Status |
|---------|--------|
| thorp_bringup | partial: velocity multiplexer and depth to scan parameters, RViz configs (see below) |
| thorp_description | migrated |
| thorp_moveit_config | migrated (see below) |
| thorp_msgs | migrated |
| thorp_cannon | migrated; simulation only, the real cannon waits for the boards (see below) |
| thorp_manipulation | partial: pickup and place object servers, fake gripper joint states (see below) |
| thorp_bt_cpp | partial: runner, `bt_server`, tests, manipulation, perception, navigation, tables and exploration nodes and trees (see below) |
| thorp_apps | partial: common launch, `pickup_objects`, `object_manip`, `patrol_2_points`, `cleanup_table`, `explore_house` and `object_gatherer` (see below) |
| thorp_costmap_layers | migrated (see below) |
| thorp_exploration | migrated, reimplemented (see below) |
| thorp_rviz_plugins | migrated (see below) |
| thorp_perception | partial: tables and objects detection, camera field of view check (see below) |
| thorp_navigation | partial: Nav2 configuration, trees and launch, maps, velocity display (see below) |
| thorp_simulation | partial: Gazebo Harmonic launch, worlds but `small_house`, controllers and navigation (see below) |
| thorp_toolkit | partial: core C++ and Python modules (see below) |
| thorp_boards, thorp_mbf_plugins, thorp_smach | ROS 1 (ignored) |

### thorp_moveit_config

MoveIt 2 configuration built with `moveit_configs_utils`, launched with `move_group.launch.py` and
`moveit_rviz.launch.py` (both take `simulation` and `use_sim_time`). It keeps Noetic's SRDF, joint velocity limits (plus
the 1.0 rad/s² accelerations MoveIt assumed), trajectory execution tolerances and OMPL, and drives `arm_controller` and
`gripper_controller` (`gripper_cmd`). `pick_ik` replaces the IKFast (TranslationDirection5D) plugin generated for the
TurtleBot arm; LMA, what ROS 2 configurations for similar arms use, is no longer in MoveIt on Jazzy. `move_group` loads
MoveIt Task Constructor's ExecuteTaskSolution capability, for `thorp_manipulation`. It publishes the semantic
description, but not the robot description, left to `robot_state_publisher`: its copy lacks the ros2_control parameters,
and Gazebo could spawn Thorp from it. The unused CHOMP and Pilz pipelines aren't ported. The Xtion octomap is disabled
(see the known issues).

### thorp_msgs

Ported from the `bt_server` branch, so it includes `RunSubtree.action`. The unused `KeyboardInput` message is
dropped, as ROS 2 rejects its lowercase constants. `DetectTables.action` is new: it replaces the executive's direct
use of RAIL segmentation to find tables, returning them as collision objects. So are the exploration planner's
services, `SegmentRooms`, `PlanRoomSequence` and `PlanRoomExploration`, with the `Room` message, in place of
`ipa_building_msgs`' actions. The interfaces of the servers the port drops are removed: `MakePickupPlan` (with
`PickupPlan`), `MoveToTarget`, `UserCommand`, the tray services and `ConnectWaypoints`. `FollowPose.action` waits for
Block 9d's `follow_pose`, that may use `opennav_following`'s action instead.

### thorp_perception

`object_detection` segments the table in front of the robot on the Xtion point cloud, and the objects on it, and
identifies each object by matching it with a 2D ICP against the templates in `meshes`. The `object_detection` action
(`thorp_msgs/DetectObjects`) adds the table and the identified objects to the planning scene, keeping the names of
redetected objects and removing those no longer on the table, but those on the tray, that can be over the table's volume
when docked under its eaves (it reads the tray geometry, `thorp_description`'s `tray.yaml`, as the executive does);
`object_detection/detect_tables` (`thorp_msgs/DetectTables`) only segments the table, if its shortest side reaches a
minimum. Tables and objects are collision objects: tables are boxes with x along their longest side; objects carry their
template mesh, and their size and color name as JSON in `type.db`, as on the `bt_server` branch. `xtion_fov_analyzer.py`
is ported too.

The segmentation and template matching are rewritten into `thorp_perception` from Thorp's forks of
`rail_segmentation` and `rail_mesh_icp`, instead of porting the forks as source dependencies: the forks carry much
dead RAIL code, and running all in one node drops the RAIL messages, the segmentation service and the template
matching action. The README records the source commits and licenses. Only what Thorp used is kept: a single crop box
instead of segmentation zones, the table as the largest quadrilateral in the surface's convex hull (now centered on
that quadrilateral rather than on the points' bounding box), and the 2D non-linear ICP; RAIL's color features,
images, markers and object recognition fields are dropped. ORK and YOLO pipelines are dropped too. Pending ROS 1
files:

| Files | Block |
|-------|-------|
| COB object detection: `config/cob`, `nodes/object_tracking.py` | 9 |
| `scripts/generate_meshes.*`, `scripts/parametric_star.scad`: offline OpenSCAD tools that made the object meshes | undecided |

### thorp_toolkit

Applications must give the toolkit their node once, with `thorp::toolkit::init(node)` in C++ or
`thorp_toolkit.common.init(node)` in Python; its singletons (`TF2`, Python's `Visualization`) and functions use it.
`common` comes from the `bt_server` branch. `tf` types are replaced by their `tf2` equivalents, overlay texts use
`rviz_2d_overlay_msgs`, and Python functions raise `ValueError` on invalid inputs and `RuntimeError` on TF failures,
instead of `rospy.ROSException`.

Ported: C++ `common`, `geometry`, `math`, `parameters`, `progress_tracker`, `tf2`, `tray` (the tray's slots, from the
planning scene objects on it, for the executive and perception) and `visualization`; Python `common`, `decorators`,
`geometry`, `progress_tracker`, `singleton`, `spatial_hash`, `tachometer`, `transform` and `visualization`, with
`test_progress_tracker`. Pending ROS 1 files, ported with their first consumer:

| Files | First consumer | Block |
|-------|----------------|-------|
| `reconfigure`, `alternative_config` (C++ and Python), `test_reconfigure.py` | navigation | 6 |
| `nodes/save_pose_node.cpp` | real robot navigation | real robot |
| `planning_scene` (C++ and Python); `simulation` (`waitForObjectsSpawning`) | tray manager, BT runner | 9 |
| `point_tracker.py` | object tracking | 9 |
| `kobuki_base` (bumper names), `semantic_map.py`, `test_semantic_map.py` | BT conditions, smach states | 9 |
| `spatial_hash.hpp` | none: an unused draft with a `main`; perception and costmap layers have their own | undecided |
| Python `pause_gazebo` and `resume_gazebo` | none | undecided |

`wait_for_mbf` is dropped with Move Base Flex, and `setup.py` goes when the package is complete.

### thorp_cannon

`cannon_ctrl.py` supports only simulation: the real cannon is commanded through the arbotix board, not ported yet. On
simulation it tilts the cannon with `cannon_joint_controller`, and fires by publishing `std_msgs/Bool` on
`arbotix/cannon_trigger` (it was an `arbotix_msgs/Digital`), bridged to Gazebo. `thorp_cannon_system`, a Gazebo
Harmonic system, replaces the Gazebo Classic plugin: while the trigger is on, it places the `rocket` model at the
cannon muzzle and launches it at the speed the configured force gives it in one simulation step. Firing needs a model
named `rocket` in the world; the cats mode of `spawn_gazebo_models.py` and the cats controller spawn it (Block 9).

### thorp_manipulation

`manipulation.launch.py` runs `move_group` and `manipulation_node`, with the `manipulation/pickup_object` and
`manipulation/place_object` action servers (`thorp_msgs`). Each goal becomes a MoveIt Task Constructor task, planned
by the node on a snapshot of `move_group`'s planning scene and executed by `move_group`'s ExecuteTaskSolution
capability; its feedback is the stage being executed (`planning`, `open gripper`, `move to pick`, `approach object`,
`close gripper`, `retreat`...). Canceling a goal stops planning or execution. Objects to pick must be in the planning
scene; they are attached to `gripper_link` on pickup, and detached on place. Grasp and place poses are Noetic's: yaw
towards the target, pitch from the distance and height, with small pitch variations as alternatives, plus the real
arm's compensations (backlash, gripper asymmetry, distance fall short) as parameters. The gripper closes to the object
side across it minus `tightening`, converted into a `gripper_joint` angle with Noetic's one-sided gripper model;
`max_effort` is ignored, as `move_group` sends its controller's fixed maximum effort. Place poses are gripper targets,
as on Noetic, and need some clearance over the support surface: contacts with it are allowed only after the place pose
IK. To release the object, the gripper opens only 1.5 cm wider than it, not to hit objects around, as those already on
the tray. Tasks plan from the measured state with its velocities cleared and its positions clamped to the joint limits:
MTC checks the start state bounds, velocities included, and pickups failed now and then with the start state out of
bounds, just after closing the gripper. On shutdown, a goal being executed stops waiting for its result, so the node
exits cleanly.
`scripts/test_pick_and_place.py` picks and places a cube on a table that exist only in the planning scene.

The fake gripper joints state publisher (`fake_joint_pub.py`, from `turtlebot_arm_bringup`) and
`launch/includes/arm.launch.py`, that runs it, are ported too. The gripper command action comes from
`ros2_controllers`' `GripperActionController` instead of Thorp's `arbotix_ros` gripper controller: goals are
`gripper_joint` angles, not openings in meters, and the action is `gripper_controller/gripper_cmd`. Pending ROS 1
files:

| Files | Block |
|-------|-------|
| Real robot servos: `param/controllers.yaml`, `launch/includes/controllers.launch.xml`, `nodes/dynamixel_*.py`, and their use in `launch/includes/arm.launch.xml`; `fake_servos_srv.py` from `thorp_bringup` on simulation | real robot |

The tray manager, the drag and drop server (`interactive_manip_server.cpp`) and the pickup planner
(`pickup_planner_server.py`) are replaced by `thorp_bt_cpp`'s tray, `DragAndDrop` and `PlanPickupLocations` nodes. MTC
exports the full planning scene, not a diff, for sub-trajectories whose end scene isn't a child of their start one;
`move_group` applied it on execution, reverting the world to its state when planned, and so removing objects detected
while executing, as `pickup_objects` does while placing on the tray. The server keeps just the robot state of those
full scenes, as the task changes the world only on its scene modifying stages, exported as diffs.

### thorp_bt_cpp

`bt_runner.launch.py` runs an app's tree, `bt/<app_name>.xml`'s root tree `<app_name>`, on a node named after the app
(given as an argument, as a launch node name would rename the nodes some tree nodes create too); with
`on_exit_shutdown`, the whole launch shuts down when the tree completes.
`bt_server.launch.py` offers every installed tree as a capability on `bt_server/run_subtree` (`thorp_msgs/RunSubtree`),
with inputs and outputs as JSON. Both load the app parameters, undeclared, and the tray geometry, from
`thorp_description/config/tray.yaml`, now a ROS 2 parameters file. `thorp::toolkit` is initialized with their node, and
subtree blackboards get the root's node and timeouts, as they see only their remapped ports.

Navigation uses `nav2_behavior_tree`'s own nodes, loaded from its plugins: `NavigateToPose`, `NavigateThroughPoses`,
`FollowPath`, `Spin` and `BackUp`, their builders wrapped to share the root entries with subtrees as Thorp's do. Their
error codes, `uint16`, go to `{nav_error}` keys, as trees can't mix types on a key and Thorp's nodes write `int` errors.
The runner waits for Nav2 to be active before ticking, when `lifecycle_manager_navigation` is running: `bt_navigator`
rejects goals until activated. Thorp's `LookAtPose` turns with the `spin` behavior; `GetPosesAroundTable` discards poses
on obstacles with the global costmap's `get_cost` service, checking the pose's cell rather than Noetic's footprint
shrunk by 10 cm, as pickup poses are under the table's eaves; `TableAsObstacle`, `ClearTableAccess` and
`RestoreTableAccess` call the semantic layer services; `PlanPickupLocations` is the pickup planner, in C++, keeping its
travel optimized order rather than sorting it again by object count as Noetic did (a TODO there questioned it), and
showing the plan on RViz as Noetic's did, on the executive's `pickup_plan` and `pickup_poses`; `PoseAsPath` makes a
straight path from a start pose, for docking. `cleanup_table` validates the table in front when given none, so the
`cleanup_table` app runs it as is.

Exploration calls the exploration planner's services: `SegmentRooms`, `PlanRoomSequence` and `PlanRoomExploration` pass
only room ids and poses, as the planner keeps the map and the rooms, instead of Noetic's map images on the blackboard.
`full_coverage_planner` plans the whole map at once with `PlanRoomExploration`, instead of `GetPath`'s Spiral-STC
coverage path, and both exploration trees drive through the poses planned with `follow_waypoints`, as on Noetic, so the
camera looks where the robot goes rather than along the planned headings. `TableVisited` asks the global costmap's
semantic layer for a table `TableAsObstacle` marked overlapping the one detected. `validate_table` goes on with the
table it's given if detecting it again, after turning to face it, misses, as a far or small table can be at the
detection limit; that livelocked `object_gatherer`, the table detected while exploring and missed once in front.
`attach_to_table` fails if docking takes over a minute, and `object_gatherer` goes on exploring if a table's cleanup
fails.

Nodes calling actions derive from Nav2's `BtActionNode`, with their servers' names as `server_name` defaults; the
runner puts Nav2's timeouts on the blackboard: `server_timeout` half a tick and `wait_for_service_timeout` 5 s, 60 s
in `bt_runner.launch.py`, as it starts with the servers. Nav2 creates action clients with the tree, so tree creation
fails if a server is missing. Services use a synchronous node template, with its own callback group. BT 4 checks
literal port values when loading a tree, so the trees drop Groot's empty values for unset ports.

Poses are written in the trees in Nav2's `PoseStamped` format (`nav2_behavior_tree` defines the string conversion);
`bt_server` takes Thorp's `x;y;yaw;frame` and `x;y;z;roll;pitch;yaw;frame` strings in its JSON inputs, and reports
poses as `{x, y, z, roll, pitch, yaw, frame}` objects. Tables are collision objects, reported as
`{name, depth, width, height, color, pose}`, and objects as `{name, color}`.

Noetic's housekeeping services, and the servers for user commands and drag and drop, are gone:

- `GripperBusy` closes the gripper to 0.45 rad: it holds something if it stalls short of 0.42.
- `ObjectAttached`, `DetachObject` and `ClearPlanningScene` use MoveIt's planning scene interface; clearing the
  gripper is a `clear_gripper` subtree (open, then detach and remove whatever was attached).
- `SetArmConfig` sends `move_group` a joint goal from the SRDF named states, read from `robot_description_semantic`.
- `ReadUserCommands` takes the commands from the `user_command` topic (`std_msgs/String`), as the RViz user commands
  panel publishes them, instead of serving `jsk_rviz_plugins`' command service; it still echoes them as an overlay
  text, on `rviz/user_command`.
- `DragAndDrop` serves the `move_objects` interactive markers itself, instead of calling a drag and drop action server
  in `thorp_manipulation`. Its place pose is the gripper's, as the place action takes it: the object's top once resting
  at the drop position, plus `placing_height_on_table`.
- The tray has no state: `NextPoseOnTray` and `TrayFull` find the free slots from the planning scene objects on it.
  Placing poses are the gripper's, so the pose on the tray is raised by the held object's height, over
  `placing_height_on_tray` (0.01 m over the tray frame, at the bottom of its 2 mm base). Noetic placed the gripper at a
  fixed 3 cm, lower than the tops of the 3.2 cm objects once on the tray; MoveIt 1's place allowed touching the tray.

`bt_server` runs one goal at a time, and is free again before it reports a goal's result, so a caller can send the
next goal as soon as it has it. Both executives take parameters the trees read at any time, declared or not. A node
throwing ends the goal, or `bt_runner`'s tree, with failure.

The tests in `test/` run against a running `bt_server`, and cover the trees installed, the ported ones:
`test_bt_server.py`, `bt_server`'s contract, on the `test_server` trees; `test_capabilities.py`, every capability in
`config/capabilities.yaml`, with the stacks named in `--stack` running; and `test_capabilities_match_trees.py`, the yaml
against the trees and `bt/node_models.xml`, now generated by the ROS 2 runner (its `nodes_filepath` parameter). The
capabilities of the trees not ported yet are skipped. Before each capability test, the `test_reset_scene` tree (in
`bt/test_capabilities.xml`) empties the gripper, rests the arm and clears the planning scene, and the test puts every
Gazebo model back to its pose at the start of the session, instead of Gazebo Classic's world reset; the session also
does it at its end, leaving the world as it found it. `--given` names the states the world already provides, sparing
the setup steps that would establish them, as `at_a_table` on the playground world:

```bash
cd ~/colcon_ws/thorp/src/thorp/thorp_bt_cpp
python3 -m pytest test/ -v --stack manipulation,perception --given at_a_table
```

With the manipulation and perception stacks up (`object_manip.launch.py executive:=llm`), all 129 tests pass.
`scripts/test_bt_server.py`, a duplicate of `test/test_bt_server.py`, is removed. `show_bt_node_on_rviz.py` shows the
running tree's action or subtree as an RViz overlay text, on `rviz/executive_progress_overlay`, and `groot/thorp.btproj`
opens the ported trees on Groot2.

Ported: the runner, `bt_server`, the JSON blackboard, the ROS logger, the manipulation, perception, list, blackboard and
parameter nodes, the user commands and drag and drop nodes, the navigation, tables and exploration nodes, the
`manipulation`, `perception`, `navigation`, `tables`, `exploration`, `pickup_objects`, `object_manip`,
`patrol_2_points`, `cleanup_table`, `explore_house`, `object_gatherer` and `test_server` trees, the tests and the
executive visualization. Removed, replaced as described or by Nav2's nodes: `add_object_to_tray`, `clear_gripper`,
`go_to_pose`, `exe_path`, `smooth_path`, `recovery`, `clear_costmaps`, `use_named_config`, `clear_rail_markers` and
`get_path`; `pickup_object` reports the object it attached from the planning scene, as Noetic's gripper busy service
did. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| `bumper_pressed`, `switch_safety_controller` | real robot |
| `cannon_control`, `cannon_has_ammo`, `monitor_objects`, `follow_pose`, `target_reachable`; `bt/cat_hunter.xml` | 9d |

### thorp_apps

`launch/includes/apps_common.launch.py` starts the simulation (Gazebo only, until the real robot's bringup is ported)
and the executive: `bt_runner` with the app's tree (`executive:=bt`, the default), or `bt_server` (`executive:=llm`).
With `viz_executive`, the executive's running node shows on RViz, and Groot2 starts, if installed in `~/Groot2`, to
monitor the tree; it isn't respawned, as Noetic's was. It also starts RViz, with the MoveIt plugin, showing the
`thorp_bringup` configuration each app passes as `rviz_config` (`rviz:=false` to skip it); the included launch files
start none, as launch arguments are global: a default in an include can't override the app's `rviz`. SMACH executives
and video recording aren't ported. `param/apps_config.yaml` is a ROS 2 parameters file for whichever node runs the app.
`pickup_objects.launch.py` adds a static `map` to `odom` transform (no map server), the objects spawner, perception and
manipulation, through `launch/includes/tabletop_manip.launch.py`, shared with `object_manip.launch.py`. `object_manip`
shuts everything down on the exit command, as on Noetic. Both show `manipulation.rviz`, with the user commands panel.
`patrol_2_points.launch.py` and `cleanup_table.launch.py` add Nav2 through `launch/includes/navigation.launch.py`, on
the playground world: the patrol between two of its open areas, around its obstacles; the cleanup on the random table
and objects of `tabletop_manip.launch.py`, without its static map; they show `navigation.rviz` and `gathering.rviz`.
`explore_house.launch.py` and `object_gatherer.launch.py` add the exploration planner, on the fun house world, with
Thorp starting at its center: the first covers the house with the Kinect's field of view, as default, showing
`exploration.rviz`; the second with the Xtion's, that detects the tables, and spawns random tables with random objects
(`fun_house_objects`) and runs perception and manipulation, showing `gathering.rviz`. Noetic's object gatherer ran on
the small house world, not ported. The executive's includes are scoped, as their `params_file` argument would otherwise
reach Nav2's launch. Pending: the other apps' ROS 1 launch files and `resources/movie_scripts`, with their blocks.

### thorp_costmap_layers

The semantic layer, marking known objects (tables, that the scans can't see well) and clearing areas (a table's
approach), is reimplemented on Nav2 rather than ported: named rectangles, added and removed through the layer's
`update_objects` service and listed by `query_objects`, as on Noetic, kept on the map frame and painted on every update,
in increasing precedence, with the types' cost, fill and padding as parameters. The layer isn't clearable, so costmap
clearing keeps the objects. Dropped with the ROS 1 implementation: its pluggable interfaces, the spatial hash (for a
handful of objects), dynamic reconfigure, forcing a costmap update, the debug markers, and the Python client, clearing
script and test.

### thorp_exploration

The exploration planner (`nodes/exploration_planner.py`, with the algorithms in the `thorp_exploration` Python module)
replaces `ipa_coverage_planning`, with simpler algorithms, on numpy, SciPy and OpenCV: rooms are seeded by eroding the
free space until regions split off below a maximum area, and grown back over it, as ipa's morphological segmentation;
rooms are ordered by nearest neighbor, improved with 2-opt, on path lengths over the free space, instead of Concorde's
exact solution; and each room's viewpoints are chosen greedily, each time the pose whose camera field of view sees the
most cells not seen yet, with walls blocking the view, until seeing 95% of what can be seen, instead of ipa's convex
set-cover program. The fields of view are Noetic's, for the Kinect or the Xtion. `test/test_planner.py` checks the
planner on the `simple_rooms` and `fun_house` maps.

### thorp_rviz_plugins

The user commands panel (`thorp_rviz_plugins/UserCommands`) replaces `jsk_rviz_plugins`' robot command buttons: its
buttons, set in the RViz configuration as a list of name, icon and command, publish the command on the `user_command`
topic. The clear costmaps tool (`thorp_rviz_plugins/ClearCostmapTool`) is a toolbar button calling both Nav2 costmaps'
`clear_entirely` services. The cancel navigation tool and the tools' overlay messages are dropped: Nav2's RViz panel
cancels navigation.

### thorp_bringup

Ported: `param/cmd_vel_mux.yaml` (was `vel_multiplexer.yaml`, now for `twist_mux`), the Kinect and Xtion depth to laser
scan parameters, `rviz/navigation.rviz` (based on Nav2's default view) and `rviz/manipulation.rviz` (based on MoveIt's,
plus the objects' interactive markers, the user command and executive overlays and the user commands panel, with the
`rviz/icons`). The other RViz configurations are built on those two, with Noetic's displays on the Jazzy topics:
`exploration.rviz`, `gathering.rviz`, `hunting.rviz`, `perception.rviz`, `view_all.rviz` and `view_model.rviz`. Their
displays for dropped packages (Move Base Flex planners, RAIL, COB, ORK, the Senz3D and external cameras, the semantic
map) are left out, and `hunting.rviz`'s detector displays wait for Block 9d. `hunting.movie.rviz` waits for video
recording, as its camera animation needs `rviz_animated_view_controller`, with no Jazzy release.
`scripts/user_commands.py` and `rviz/user_commands.yaml` are replaced by the user commands panel and
`ReadUserCommands`. Everything else is pending: real robot launch files and drivers, scripts and the docker image.

### thorp_navigation

Ported: `navigation.launch.py`, with Nav2 in place of Move Base Flex and the Noetic arguments (localization `amcl`,
`static` or `gazebo`, map, initial pose, velocity smoothing), `param/nav2.yaml` and the maps, except `small_house`
(symlinks to a `small_house_world` package checkout). Velocity commands keep the Noetic topics: navigation, through the
velocity smoother if enabled, into `cmd_vel_mux/input/navigation`; `twist_mux` uses unstamped velocities, as Nav2 and
the Kobuki base. The controller runs at 20 Hz instead of Noetic's 15, as MPPI needs a control period no longer than its
model step. For docking at tables, a precise controller (`PreciseFollowPath`, Regulated Pure Pursuit, slow and turning
in place) and goal checker (`precise_goal_checker`, Noetic's 3.5 cm and 0.05 rad) replace the `precise_controlling`
named configuration; it steers for a point beyond the end of the path (`interpolate_curvature_after_goal`), as steering
for a goal a few centimeters off its heading made it circle around it; the executive picks them per `follow_path` goal.
As the controller server rejects goals naming no goal checker when it has more than one, `bt_navigator` runs
`behavior_trees/`: Nav2's default trees, selecting also the goal checker, the general one by default. Both costmaps have
the semantic layer, before inflation. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| Robot pose saving (`thorp_toolkit`'s `save_pose_node`): Nav2's AMCL doesn't keep its pose across restarts | real robot |
| `src/pose_servoing.cpp`, `src/visual_servoing.cpp` (not built on Noetic either) | undecided |

Replaced by Nav2 packages: the pose follower (`src/pose_follower.cpp`, `cfg/Follower.cfg`, `param/pose_follower.yaml`)
by `opennav_following`'s `follow_object` action, with the desired distance and detection timeout as its parameters instead
of per goal, and without stopping at the distance (goals end on `max_duration` or cancel); following commands go to
`cmd_vel_mux/input/following`. The waypoints path (`src/waypoints_path.cpp`, `param/waypoints_path.yaml` and its
`ConnectWaypoints` service) by the planner's `compute_path_through_poses` and the smoother server's `smooth_path`.
`nodes/show_velocity.py` is ported, displayed by `navigation.rviz` with `rviz_2d_overlay_plugins`.

Dropped with Move Base Flex: `param/move_base_flex/`, `launch/includes/move_base_flex.launch.xml`,
`nodes/mbf_simple_goal_relay.py` (Nav2 takes RViz goals itself) and the MBF test scripts in `scripts/test/`. The
`virtual` obstacle source is dropped too; nothing published it.

### thorp_simulation

Ported: `thorp_gazebo.launch.py` and `navigation.launch.py`, the `empty`, `playground` and `fun_house` worlds (its
house, the `fun_house` model, is Thorp's edit of `turtlebot3_gazebo`'s, with that package's textures), the Gazebo
models, the ROS / Gazebo bridge configuration, the controllers configuration and `gazebo_ground_truth`, now fed by
Gazebo's odometry publisher. `thorp_gazebo.launch.py` also runs what Noetic's `sim_common.launch.xml` and
`thorp_gazebo.launch.xml` did: velocity commands multiplexer and depth image and point cloud to laser scan.
`spawn_gazebo_models.py` populates the world with tables and objects through Gazebo's create service, bridged to ROS
with the remove and set pose services: the playground modes (`playground_fixed`, `playground_cubes`, `playground_rows`,
`playground_random`) put a table in front of the robot, and `fun_house_objects` spawns random tables with objects in
open spaces of any map, checked on Nav2's global costmap instead of Move Base Flex's check pose service. Objects are
placed relative to their tables by the script, as Gazebo can't place them relative to a model created on the same step.
Its `-d` option, to delete previously spawned models, is dropped; it didn't work on Noetic. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| Bumper and cliff point clouds in `thorp_gazebo.launch.xml` | real robot |
| `spawn_gazebo_models.py` cats (with the rocket) mode | 9d |
| `spawn_gazebo_models.py` `small_house_objects` mode, `small_house` Gazebo world | when needed |
| `src/gazebo_camera_control*.cpp`, `nodes/` (cats controller, model markers, movie director) | 9 |
| Stage and STDR launch files, worlds and robot configurations (no Jazzy release of either simulator) | undecided |
| `scripts/gazebo_link_state.py` (`gz model -m thorp -l <link> -p` shows the same) | undecided |

## Simulation on Gazebo Harmonic

```bash
ros2 launch thorp_simulation thorp_gazebo.launch.py [world_name:=empty] [gui:=false]
ros2 launch thorp_simulation navigation.launch.py [localization:=gazebo] [visualization:=false]
```

Differences with the Noetic simulation:

- The Kobuki base uses Gazebo's `DiffDrive`, `JointStatePublisher` and `Imu` systems instead of the `kobuki_gazebo`
  plugin, on the same topics. It has no command timeout, and no bumpers, cliff or wheel drop sensors; bumpers and
  cliff sensors come with Block 6, where navigation needs them.
- Gazebo Harmonic has no sonar sensor. Sonars and IR sensors are GPU lidars, whose scans `ros_gz_bridge` converts
  into range messages with the closest reading of all rays. It reports `INFRARED` radiation for the sonars too, and
  `max_range + 1` when nothing is in range. IR sensors use three rays, to report their field of view.
- The center sonar publishes on `mobile_base/sensors/sonars/p0`, as ROS 2 names can't start with a digit.
- Point clouds are created from the depth images by `depth_image_proc`, as Gazebo's use the camera link axes.
- Arm, gripper and cannon servos use `ros2_control` on Gazebo (`gz_ros2_control`), starting on the resting pose.
  `gz_ros2_control` turns position commands into joint velocities proportional to the error; its gain is raised to 1.0,
  as with the default 0.1 the arm lags MoveIt trajectories beyond the controller tolerances. Even so, under load the
  shoulder lift occasionally lags more than Noetic's 0.1 rad path tolerance, so `arm_controller` has no path tolerance
  in simulation; goals must still be reached within 0.1 rad. It starts from the measured joint positions, as its
  command interfaces start at 0 instead of the resting pose, and accepts trajectories ending with tiny velocities, as
  MoveIt Task Constructor's Cartesian paths do.
  `arm_controller` provides the same `arm_controller/follow_joint_trajectory` action; `gripper_controller` provides
  `gripper_controller/gripper_cmd`, taking `gripper_joint` angles; the cannon position controller is
  `cannon_joint_controller`, as ros2_control controllers can't be named as their joints. Simulated servos report no
  effort, so the gripper only detects stalls by not moving.
- No grasp-fix plugin: friction holds grasped objects well enough, as the gripper keeps pressing them. Pressing an
  object, the simulated servo chatters, so `gripper_controller` detects stalls in 0.1 s under 0.01 rad/s; Noetic's
  0.5 s takes seconds, beyond MoveIt's execution time limit. Without the plugin's grasp events, the house keeping's
  gripper busy check needs another way to tell whether an object is held in Block 9: e.g. the gripper stopping before
  its closing target.
- `thorp_gazebo.launch.py` runs Gazebo itself, with the paths `ros_gz_sim`'s `gz_sim.launch.py` sets, as that one
  runs it through a shell, that doesn't pass launch's signals on: Gazebo kept running when launch shut down other
  than with Ctrl-C, as on an app's exit command.
- Gazebo publishes the clock on every step (1 kHz); `thorp_gazebo.launch.py` throttles it to 100 Hz, as Noetic's
  `gazebo_ros` did, as sim time Python nodes need a lot of CPU to process it at 1 kHz.
- `thorp_gazebo.launch.py` sets Gazebo transport on loopback (`GZ_IP=127.0.0.1`): with a VPN interface (Tailscale)
  on the development machine, a quarter of the simulations started without clock, as Gazebo missed the bridge's
  subscription to it; none of 30 on loopback.

## Known issues

- Range messages from `ros_gz_bridge` report `max_range + 1` when nothing is in range (hardcoded in its LaserScan to
  Range conversion), and Nav2's range layer discards readings above `max_range`, before applying `clear_on_max_reading`.
  So in simulation, sonars and IR sensors never clear the costmaps with "nothing in range" readings: their marks
  persist until an in-range reading clears them, or the costmaps are cleared. The range layers' `no_readings_timeout`
  is disabled; otherwise, without valid readings, they make the costmaps not current, blocking the planner and
  controller. A proper fix would be an option in `ros_gz_bridge` to report `max_range` instead
  (https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/sensor_msgs.cpp#L556); reported upstream
  in https://github.com/gazebosim/ros_gz/issues/959.
- MoveIt's Xtion octomap gets voxels in contact with the resting gripper, so every planning request starts in collision.
  The raw depth images are right (the gripper is closer than the near clipping distance, so it's not in them) and the
  self-filter renders the robot in place, so the voxels come from elsewhere; the octomap sensor is disabled until it's
  investigated with the manipulation servers or perception.
- In Gazebo, the arm sometimes stalls on some poses that MoveIt finds collision free (about 1 in 40 random arm goals in
  testing, e.g. `[0.16, 1.14, 1.57, 1.47]`, with the gripper 0.39 m above the floor): the shoulder lift lags up to 0.9 rad
  and MoveIt stops the execution as timed out, leaving the arm off its path, sometimes in collision for the next plan.
  Maybe a physical contact MoveIt doesn't model (the arm links have `selfCollide`); to check with the manipulation
  servers.
- The simulated Kobuki rests tilted back on its back caster, 2 mm above the ground in the model, by 0.015 rad
  (0.85 degrees) while the arm rests: its center of mass must be behind the wheels. TF assumes a level base, so
  points seen by the cameras are about 9 mm off at the tables distance (farther and lower); the tilt goes away when
  the arm reaches forward. Detection and grasping still work.
- In simulation, `xtion_fov_analyzer.py` reports the field of view blocked with the arm resting: the Xtion sees
  something at 0.37-0.40 m just above the cropped bottom of the image, probably the resting arm, as the simulated
  depth near clip is 0.35 m (0.45 m on the real camera). Maybe related to the octomap issue above.
- MoveIt's `PlanningSceneInterface::getKnownObjectNamesInROI` takes the collision objects' shape poses, relative to
  the object pose, as absolute, so it misses objects placed with a pose; `object_detection` checks the objects' poses
  itself instead. Fixed upstream by https://github.com/moveit/moveit2/pull/3884 (open); once released on Jazzy,
  `objectsInVolume` can go back to it.
- In simulation, grasps across an object's wide side (openings of 2.6 to 3.2 cm) often fail: after closing, the gripper
  holds nothing. Across the 1.1 cm side, they succeed. The arm has no wrist roll, so objects whose thin side faces the
  arm can only be grasped across the wide one. In a `pickup_objects` run, 4 objects ended on the tray and `clover 1` was
  given up after failing like this 3 times; in a `cleanup_table` run, 3 of 4 ended on the tray and a triangle was given
  up.
- On the playground, the patrol's first leg stalled in testing: the controller aborted with "Failed to make progress"
  until Nav2's recoveries got it moving. It looks like MPPI tuning, not the trees.
- In an `object_gatherer` run, one docking in ten made no progress, and the progress checker aborted it: on a 0.62 m
  table side split into 3 pickup locations. Maybe a table leg blocks the robot near a corner, as `GetPosesAroundTable`
  only checks the pose's own cell on the costmap, the pickup poses being under the table's eaves; not confirmed.
- Tray slots are 3.5 cm apart, and objects on the tray stay in the planning scene, so placing next to a wide object
  (a cross, say) often fails: a gripper finger, or opening the gripper, collides with it. In an `object_gatherer` run,
  7 of 12 placements failed planning like this.
- `move_group` segfaults on Ctrl-C, in `TrajectoryExecutionManager`'s destructor: MoveIt's
  https://github.com/moveit/moveit2/issues/3680, with a fix in https://github.com/moveit/moveit2/pull/3828 (open).
