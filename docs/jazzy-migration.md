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
  src/third_party/    # source dependencies from thorp-jazzy.repos (none yet)
```

Build:

```bash
cd ~/colcon_ws/thorp
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

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
| 6c | Semantic costmap layer | deferred to 9 |
| 6d | Bumpers and cliff sensors, on simulation and costmaps | deferred to real robot |
| 6e | Coverage planning | deferred to 9 |
| 7a | MoveIt 2 configuration: move group, controllers, octomap from the Xtion, RViz; pick_ik replaces the IKFast plugin | done |
| 7b | Pickup and place object action servers on MoveIt Task Constructor | done |
| 7c | Grasping in simulation and object spawning | done |
| 8 | Perception: tables and tabletop objects detection | done |
| 9 | Executive: behavior trees and apps | |

Block 3 onwards will be refined as we get there.

Navigation uses Nav2 only; Move Base Flex is dropped, with whatever depends on it (`thorp_mbf_plugins`, MBF actions
and plugins configuration in `thorp_navigation`). The executive will use Nav2's own interfaces (`navigate_to_pose`,
`compute_path_to_pose`, `follow_path`, behaviors and the costmaps' `get_cost` service). MPPI replaces TEB, which has no
Jazzy release, and Nav2's standard behaviors replace `SlowEscapeRecovery` until the executive shows whether they are
enough.

The semantic costmap layer (`thorp_costmap_layers`) is only used by the executive, to mark tables as obstacles in both
costmaps (the scans can't see their eaves) and to clear an approach area in front of them in the local costmap. Its
port is deferred to Block 9, where the executive shows what the table approach needs. Nav2 on Jazzy can mark regions
with its keepout filter, fed with a mask of rectangles by a small node, but can't clear regions persistently (only once,
with `clear_around_pose`); porting the layer as a Nav2 plugin keeps the Noetic behavior.

Bumpers and cliff sensors are deferred to the real robot: it needs `kobuki_ros` from source anyway (the Kobuki driver,
`kobuki_bumper2pc` and `kobuki_safety_controller` have no Jazzy release), and simulated bumper and cliff events, from
Gazebo contact sensors and downward rays, can then match the real driver's topics.

MoveIt 2 has no pick and place capability (`moveit_msgs` keeps the `Pickup` and `Place` actions, but nothing serves
them), so the pickup and place object servers build MoveIt Task Constructor tasks instead, keeping their `thorp_msgs`
actions, now with the stage being executed as feedback. The rest of Noetic's manipulation servers go to the executive,
in Block 9: `move_to_target` (the behavior trees only use named targets, that `move_group` takes directly), the house
keeping services (clear gripper, force resting, object attached and gripper busy: planning scene queries and
controller commands), the tray manager (slot poses, as executive state) and the drag and drop demo. So does the pickup
planner, that groups objects on a table into pickup locations: app-level planning, with no MoveIt use.

Coverage planning is deferred to Block 9, with the exploration apps, its only users. No option is a Jazzy release:
`ipa_coverage_planning` (room segmentation, room sequence planning and room exploration) has no ROS 2 port;
`full_coverage_path_planner`'s `ros2` branch is an unfinished migration, untouched since 2023; and `opennav_coverage`
(Nav2 coverage server, source only) needs Fields2Cover 1.2.1, while Jazzy releases 2.0.0, and plans coverage of
polygons, without room segmentation nor sequencing. Coverage planning is the only navigation code Thorp may need
beyond Nav2: on Noetic, `ipa_coverage_planning` (room segmentation, room sequence planning and room exploration, from
Thorp's fork) and `full_coverage_path_planner`'s Spiral-STC planner, as an MBF global planner.

## Package status

| Package | Status |
|---------|--------|
| thorp_bringup | partial: velocity multiplexer and depth to scan parameters, navigation RViz config (see below) |
| thorp_description | migrated |
| thorp_moveit_config | migrated (see below) |
| thorp_msgs | migrated |
| thorp_cannon | migrated; simulation only, the real cannon waits for the boards (see below) |
| thorp_manipulation | partial: pickup and place object servers, fake gripper joint states (see below) |
| thorp_perception | partial: tables and objects detection, camera field of view check (see below) |
| thorp_navigation | partial: Nav2 configuration and launch, maps, velocity display (see below) |
| thorp_simulation | partial: Gazebo Harmonic launch, worlds, controllers and navigation (see below) |
| thorp_toolkit | partial: core C++ and Python modules (see below) |
| thorp_apps, thorp_boards, thorp_bt_cpp, thorp_costmap_layers, thorp_exploration, thorp_mbf_plugins, thorp_rviz_plugins, thorp_smach | ROS 1 (ignored) |

### thorp_moveit_config

MoveIt 2 configuration built with `moveit_configs_utils`, launched with `move_group.launch.py` and
`moveit_rviz.launch.py` (both take `simulation` and `use_sim_time`). It keeps Noetic's SRDF, joint velocity limits
(plus the 1.0 rad/s² accelerations MoveIt assumed), trajectory execution tolerances and OMPL, and drives
`arm_controller` and `gripper_controller` (`gripper_cmd`). `pick_ik` replaces the IKFast (TranslationDirection5D)
plugin generated for the TurtleBot arm; LMA, what ROS 2 configurations for similar arms use, is no longer in MoveIt on
Jazzy. `move_group` loads MoveIt Task Constructor's ExecuteTaskSolution capability, for `thorp_manipulation`. The unused CHOMP and Pilz pipelines aren't ported. The Xtion octomap is disabled (see the known issues).

### thorp_msgs

Ported from the `bt_server` branch, so it includes `RunSubtree.action`. The unused `KeyboardInput` message is
dropped, as ROS 2 rejects its lowercase constants. `DetectTables.action` is new: it replaces the executive's direct
use of RAIL segmentation to find tables, returning them as collision objects.

### thorp_perception

`object_detection` segments the table in front of the robot on the Xtion point cloud, and the objects on it, and
identifies each object by matching it with a 2D ICP against the templates in `meshes`. The `object_detection` action
(`thorp_msgs/DetectObjects`) adds the table and the identified objects to the planning scene, keeping the names of
redetected objects and removing those no longer on the table; `object_detection/detect_tables`
(`thorp_msgs/DetectTables`) only segments the table, if its shortest side reaches a minimum. Tables and objects are
collision objects: tables are boxes with x along their longest side; objects carry their template mesh, and their
size and color name as JSON in `type.db`, as on the `bt_server` branch. `xtion_fov_analyzer.py` is ported too.

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

Ported: C++ `common`, `geometry`, `math`, `parameters`, `progress_tracker`, `tf2` and `visualization`; Python
`common`, `decorators`, `geometry`, `progress_tracker`, `singleton`, `spatial_hash`, `tachometer`, `transform` and
`visualization`, with `test_progress_tracker`. Pending ROS 1 files, ported with their first consumer:

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
IK. `scripts/test_pick_and_place.py` picks and places a cube on a table that exist only in the planning scene.

The fake gripper joints state publisher (`fake_joint_pub.py`, from `turtlebot_arm_bringup`) and
`launch/includes/arm.launch.py`, that runs it, are ported too. The gripper command action comes from
`ros2_controllers`' `GripperActionController` instead of Thorp's `arbotix_ros` gripper controller: goals are
`gripper_joint` angles, not openings in meters, and the action is `gripper_controller/gripper_cmd`. Pending ROS 1
files:

| Files | Block |
|-------|-------|
| Real robot servos: `param/controllers.yaml`, `launch/includes/controllers.launch.xml`, `nodes/dynamixel_*.py`, and their use in `launch/includes/arm.launch.xml`; `fake_servos_srv.py` from `thorp_bringup` on simulation | real robot |
| `src/tray_manager_server.cpp`, `src/interactive_manip_server.cpp` (drag and drop), `nodes/pickup_planner_server.py`, `setup.py` | 9 |

### thorp_bringup

Ported: `param/cmd_vel_mux.yaml` (was `vel_multiplexer.yaml`, now for `twist_mux`), the Kinect and Xtion depth to
laser scan parameters and `rviz/navigation.rviz` (based on Nav2's default view). Everything else is pending: real robot
launch files and drivers, other RViz configurations, scripts and the docker image.

### thorp_navigation

Ported: `navigation.launch.py`, with Nav2 in place of Move Base Flex and the Noetic arguments (localization `amcl`,
`static` or `gazebo`, map, initial pose, velocity smoothing), `param/nav2.yaml` and the maps, except `small_house`
(symlinks to a `small_house_world` package checkout). Velocity commands keep the Noetic topics: navigation, through
the velocity smoother if enabled, into `cmd_vel_mux/input/navigation`; `twist_mux` uses unstamped velocities, as
Nav2 and the Kobuki base. The controller runs at 20 Hz instead of Noetic's 15, as MPPI needs a control period no longer
than its model step. Pending ROS 1 files:

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

Ported: `thorp_gazebo.launch.py` and `navigation.launch.py`, the `empty` and `playground` worlds, the Gazebo models, the
ROS / Gazebo bridge configuration, the controllers configuration and `gazebo_ground_truth`, now fed by Gazebo's
odometry publisher. `thorp_gazebo.launch.py` also runs what Noetic's `sim_common.launch.xml` and
`thorp_gazebo.launch.xml` did: velocity commands multiplexer and depth image and point cloud to laser scan.
`spawn_gazebo_models.py` populates the world with tables and objects through Gazebo's create service, bridged to ROS
with the remove and set pose services: the playground modes (`playground_fixed`, `playground_cubes`, `playground_rows`,
`playground_random`) put a table in front of the robot, and `fun_house_objects` spawns random tables with objects in
open spaces of any map, checked on Nav2's global costmap instead of Move Base Flex's check pose service. Objects are
placed relative to their tables by the script, as Gazebo can't place them relative to a model created on the same
step. Its `-d` option, to delete previously spawned models, is dropped; it didn't work on Noetic. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| Bumper and cliff point clouds in `thorp_gazebo.launch.xml` | real robot |
| `spawn_gazebo_models.py` cats (with the rocket) and `small_house_objects` modes | 9, with the small house world |
| `src/gazebo_camera_control*.cpp`, `nodes/` (cats controller, model markers, movie director) | 9 |
| `fun_house` and `small_house` Gazebo worlds | when needed |
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
- Once, of about 25 simulated place goals, planning failed with "open gripper: Start state is out of bounds!"; it
  didn't happen again, so the joint out of bounds is unknown.
- The simulated Kobuki rests tilted back on its back caster, 2 mm above the ground in the model, by 0.015 rad
  (0.85 degrees) while the arm rests: its center of mass must be behind the wheels. TF assumes a level base, so
  points seen by the cameras are about 9 mm off at the tables distance (farther and lower); the tilt goes away when
  the arm reaches forward. Detection and grasping still work.
- In simulation, `xtion_fov_analyzer.py` reports the field of view blocked with the arm resting: the Xtion sees
  something at 0.37-0.40 m just above the cropped bottom of the image, probably the resting arm, as the simulated
  depth near clip is 0.35 m (0.45 m on the real camera). Maybe related to the octomap issue above.
