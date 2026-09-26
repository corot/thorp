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
| 5 | `thorp_msgs`, `thorp_toolkit`; `thorp_cannon`, with a Gazebo Harmonic firing system | next |
| 6 | Navigation: Nav2 configuration, semantic costmap layer, coverage planning | |
| 7 | Manipulation: MoveIt 2 configuration, pick and place servers, grasping on simulation | |
| 8 | Perception | |
| 9 | Executive: behavior trees and apps | |

Block 3 onwards will be refined as we get there.

Navigation uses Nav2 only; Move Base Flex is dropped, with whatever depends on it (`thorp_mbf_plugins`, MBF actions
and plugins configuration in `thorp_navigation`). Coverage planning is the only navigation code Thorp may need
beyond Nav2: on Noetic, `ipa_coverage_planning` (room segmentation, room sequence planning and room exploration, from
Thorp's fork) and `full_coverage_path_planner`'s Spiral-STC planner, as an MBF global planner.

## Package status

| Package | Status |
|---------|--------|
| thorp_description | migrated |
| thorp_manipulation | partial: gripper controller (see below) |
| thorp_simulation | partial: Gazebo Harmonic launch, worlds and controllers (see below) |
| thorp_apps, thorp_boards, thorp_bringup, thorp_bt_cpp, thorp_cannon, thorp_costmap_layers, thorp_exploration, thorp_mbf_plugins, thorp_moveit_config, thorp_msgs, thorp_navigation, thorp_perception, thorp_rviz_plugins, thorp_smach, thorp_toolkit | ROS 1 (ignored) |

### thorp_manipulation

Ported: the gripper command action server (`gripper_controller.py`, from Thorp's `arbotix_ros` fork, one-side model
only), the fake gripper joints state publisher (`fake_joint_pub.py`, from `turtlebot_arm_bringup`) and
`launch/includes/arm.launch.py`, that runs both. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| Real robot servos: `param/controllers.yaml`, `launch/includes/controllers.launch.xml`, `nodes/dynamixel_*.py`, and their use in `launch/includes/arm.launch.xml`; `fake_servos_srv.py` from `thorp_bringup` on simulation | real robot |
| Everything else: manipulation servers, `pickup_planner_server.py`, test scripts, `setup.py`, launch files and parameters | 7 |

### thorp_simulation

Ported: `thorp_gazebo.launch.py`, the `empty` and `playground` worlds, the Gazebo models, the ROS / Gazebo bridge
configuration and the controllers configuration. Pending ROS 1 files:

| Files | Block |
|-------|-------|
| `src/gazebo_ground_truth.cpp`, `scripts/gazebo_link_state.py` | 5 |
| Cannon plugin (in `thorp_cannon`) | 5 |
| `launch/navigation.launch`, `launch/includes/sim_common.launch.xml` (cmd_vel mux); depth image and point cloud to laser scan, and bumper / cliff point clouds in `thorp_gazebo.launch.xml` | 6 |
| `scripts/spawn_gazebo_models.py`; grasp-fix plugin | 7 |
| `src/gazebo_camera_control*.cpp`, `nodes/` (cats controller, model markers, movie director) | 9 |
| `fun_house` and `small_house` Gazebo worlds | when needed |
| Stage and STDR launch files, worlds and robot configurations (no Jazzy release of either simulator) | undecided |

## Simulation on Gazebo Harmonic

```bash
ros2 launch thorp_simulation thorp_gazebo.launch.py [world_name:=empty] [gui:=false]
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
  `arm_controller` provides the same `arm_controller/follow_joint_trajectory` action; the gripper and cannon position
  controllers are `gripper_joint_controller` and `cannon_joint_controller`, as ros2_control controllers can't be named
  as their joints. `gripper_controller` provides `gripper_controller/gripper_action` on top of the gripper one, as on
  Noetic. Simulated servos report no effort, so the gripper detects stalls by its lack of progress.
- No grasp-fix plugin yet: grasped objects are held only by friction.
- Gazebo prints `gz_frame_id` warnings when spawning Thorp: SDFormat 14 doesn't know this element yet, but Gazebo
  uses it to stamp sensor messages with the URDF frames. It also warns that `gripper_link` has no inertia, so
  `gripper_link_joint` is dropped from the simulated model, as it was on Noetic.
