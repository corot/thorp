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
| 6b | Thorp navigation nodes: pose follower, waypoints path, velocity display, robot pose saving | next |
| 6c | Semantic costmap layer | |
| 6d | Bumpers and cliff sensors, on simulation and costmaps | |
| 6e | Coverage planning | |
| 7 | Manipulation: MoveIt 2 configuration, pick and place servers, grasping on simulation | |
| 8 | Perception | |
| 9 | Executive: behavior trees and apps | |

Block 3 onwards will be refined as we get there.

Navigation uses Nav2 only; Move Base Flex is dropped, with whatever depends on it (`thorp_mbf_plugins`, MBF actions
and plugins configuration in `thorp_navigation`). The executive will use Nav2's own interfaces (`navigate_to_pose`,
`compute_path_to_pose`, `follow_path`, behaviors and the costmaps' `get_cost` service). MPPI replaces TEB, which has no
Jazzy release, and Nav2's standard behaviors replace `SlowEscapeRecovery` until the executive shows whether they are
enough. Coverage planning is the only navigation code Thorp may need
beyond Nav2: on Noetic, `ipa_coverage_planning` (room segmentation, room sequence planning and room exploration, from
Thorp's fork) and `full_coverage_path_planner`'s Spiral-STC planner, as an MBF global planner.

## Package status

| Package | Status |
|---------|--------|
| thorp_bringup | partial: velocity multiplexer and depth to scan parameters, navigation RViz config (see below) |
| thorp_description | migrated |
| thorp_msgs | migrated |
| thorp_cannon | migrated; simulation only, the real cannon waits for the boards (see below) |
| thorp_manipulation | partial: fake gripper joint states (see below) |
| thorp_navigation | partial: Nav2 configuration and launch, maps (see below) |
| thorp_simulation | partial: Gazebo Harmonic launch, worlds, controllers and navigation (see below) |
| thorp_toolkit | partial: core C++ and Python modules (see below) |
| thorp_apps, thorp_boards, thorp_bt_cpp, thorp_costmap_layers, thorp_exploration, thorp_mbf_plugins, thorp_moveit_config, thorp_perception, thorp_rviz_plugins, thorp_smach | ROS 1 (ignored) |

### thorp_msgs

Ported from the `bt_server` branch, so it includes `RunSubtree.action`. The unused `KeyboardInput` message is
dropped, as ROS 2 rejects its lowercase constants.

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
| `reconfigure`, `alternative_config` (C++ and Python), `test_reconfigure.py`; `nodes/save_pose_node.cpp` | navigation | 6 |
| `planning_scene` (C++ and Python); `simulation` (`waitForObjectsSpawning`) | manipulation, BT runner | 7, 9 |
| `point_tracker.py` | object tracking | 8 |
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
named `rocket` in the world; `spawn_gazebo_models.py` and the cats controller spawn it.

### thorp_manipulation

Ported: the fake gripper joints state publisher (`fake_joint_pub.py`, from `turtlebot_arm_bringup`) and
`launch/includes/arm.launch.py`, that runs it. The gripper command action comes from `ros2_controllers`'
`GripperActionController` instead of Thorp's `arbotix_ros` gripper controller: goals are `gripper_joint` angles, not
openings in meters, and the action is `gripper_controller/gripper_cmd`. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| Real robot servos: `param/controllers.yaml`, `launch/includes/controllers.launch.xml`, `nodes/dynamixel_*.py`, and their use in `launch/includes/arm.launch.xml`; `fake_servos_srv.py` from `thorp_bringup` on simulation | real robot |
| Everything else: manipulation servers, `pickup_planner_server.py`, test scripts, `setup.py`, launch files and parameters | 7 |

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
| `src/pose_follower.cpp`, `cfg/Follower.cfg`, `param/pose_follower.yaml`, `src/waypoints_path.cpp`, `param/waypoints_path.yaml`, `nodes/show_velocity.py`; robot pose saving (`thorp_toolkit`'s `save_pose_node`) | 6b |
| `src/pose_servoing.cpp`, `src/visual_servoing.cpp` (not built on Noetic either) | undecided |

Dropped with Move Base Flex: `param/move_base_flex/`, `launch/includes/move_base_flex.launch.xml`,
`nodes/mbf_simple_goal_relay.py` (Nav2 takes RViz goals itself) and the MBF test scripts in `scripts/test/`. The
`virtual` obstacle source is dropped too; nothing published it.

### thorp_simulation

Ported: `thorp_gazebo.launch.py` and `navigation.launch.py`, the `empty` and `playground` worlds, the Gazebo models, the
ROS / Gazebo bridge configuration, the controllers configuration and `gazebo_ground_truth`, now fed by Gazebo's
odometry publisher. `thorp_gazebo.launch.py` also runs what Noetic's `sim_common.launch.xml` and
`thorp_gazebo.launch.xml` did: velocity commands multiplexer and depth image and point cloud to laser scan. Pending
ROS 1 files:

| Files | Block |
|-------|-------|
| Bumper and cliff point clouds in `thorp_gazebo.launch.xml` | 6d |
| `scripts/spawn_gazebo_models.py`; grasp-fix plugin | 7 |
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
  `arm_controller` provides the same `arm_controller/follow_joint_trajectory` action; `gripper_controller` provides
  `gripper_controller/gripper_cmd`, taking `gripper_joint` angles; the cannon position controller is
  `cannon_joint_controller`, as ros2_control controllers can't be named as their joints. Simulated servos report no
  effort, so the gripper only detects stalls by not moving.
- No grasp-fix plugin yet: grasped objects are held only by friction.
- Gazebo publishes the clock on every step (1 kHz); `thorp_gazebo.launch.py` throttles it to 100 Hz, as Noetic's
  `gazebo_ros` did, as sim time Python nodes need a lot of CPU to process it at 1 kHz.

## Known issues

- Range messages from `ros_gz_bridge` report `max_range + 1` when nothing is in range, and Nav2's range layer discards
  readings above `max_range`, before applying `clear_on_max_reading`. So in simulation, sonars and IR sensors never clear
  the costmaps, and after `no_readings_timeout` (2 s) without valid readings the range layers make the costmaps not
  current, blocking the planner and controller. Navigation on simulation needs a fix for this; options are a small
  component that sets out of range readings to `max_range`, or disabling the timeout.
- Gazebo prints `gz_frame_id` warnings when spawning Thorp: SDFormat 14 doesn't know this element yet, but Gazebo
  uses it to stamp sensor messages with the URDF frames. It also warns that `gripper_link` has no inertia, so
  `gripper_link_joint` is dropped from the simulated model, as it was on Noetic.
