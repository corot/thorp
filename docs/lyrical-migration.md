# Thorp on ROS 2 Lyrical

The `lyrical` branch moves the ROS 2 port of Thorp from Jazzy to ROS 2 Lyrical, Gazebo Jetty and Ubuntu 26.04 (Python
3.14). It starts from the `jazzy` branch, whose design stands: [jazzy-migration.md](jazzy-migration.md) still describes
each package, and this document records only what Lyrical changes.

## Rules

- Target: ROS 2 Lyrical and Gazebo Jetty, on Ubuntu 26.04.
- Same robot as on Jazzy: frames, joints, topics, capabilities and apps stay as they are, unless Lyrical removes
  something they use.
- Dependencies: Lyrical binary packages first. Source dependencies go in `thorp-lyrical.repos`, pinned to a commit
  SHA; Thorp's own forks (`corot/...`) follow their branch.
- Each block is checked as on Jazzy: a clean build with no new warnings, then what the block changes, run headless.
  `thorp_description` must still produce the same URDF.

## Workspace

```text
~/colcon_ws/thorp/
  src/thorp/          # this repository, lyrical branch
  src/third_party/    # source dependencies from thorp-lyrical.repos
```

Build:

```bash
cd ~/colcon_ws/thorp
source /opt/ros/lyrical/setup.bash
vcs import src/third_party < src/thorp/thorp-lyrical.repos
rosdep install --from-paths src --ignore-src -r -y --skip-keys "python3-torchvision-pip python3-ultralytics-pip"
colcon build --symlink-install
source install/setup.bash
```

As on Jazzy, `yolo_ros` needs `uv` to create its own Python virtual environment for PyTorch and Ultralytics, hence
the skipped keys. Its wheel holds only the binary, so the user site is a safe place for it:
`python3 -m pip install uv --user --break-system-packages`. `thorp_agent` needs ROSA and LangChain, installed as its
README says.

## Blocks

| # | Block | Status |
|---|-------|--------|
| 1 | Workspace: branch, this document, `thorp-lyrical.repos`, dependencies from Lyrical binaries | done |
| 2 | Build: every package compiles on Lyrical, with no new warnings | done |
| 3 | `thorp_description`: the same URDF as on Jazzy | done, see below |
| 4 | Simulation on Gazebo Jetty: spawn, sensors, controllers, cannon, tray objects | done |
| 5 | Navigation on Nav2 1.5: localization, costmaps and their layers, controllers, behaviors | done |
| 6 | Manipulation on MoveIt 2.15: pick and place, the parallel gripper | |
| 7 | Perception: tables and objects, YOLO on Python 3.14 | |
| 8 | Behavior trees, `thorp_world` and the apps: capability tests on simulation | |
| 9 | `thorp_agent`: ROSA and LangChain on Python 3.14 | |

## Changes

| Lyrical | Thorp |
|---|---|
| tf2's `.h` headers are gone, and tf2_ros' deprecated | `.hpp` headers |
| `geometry_msgs/Pose2D` is gone | `thorp_toolkit`'s 2D pose conversions, which nothing used, are gone too |
| ros2_controllers has no `gripper_controllers` nor `position_controllers` | The gripper runs on `parallel_gripper_controller`, whose `control_msgs/ParallelGripperCommand` action MoveIt drives with its `ParallelGripperCommand` controller type, and `MoveGripper` and `GripperBusy` too; the cannon on `forward_command_controller`, with the same topic and message |
| Nav2 1.5 plugins run on `nav2::LifecycleNode`, with its parameter helpers in `nav2_ros_common`: layers declare parameters with `declare_or_get_parameter`, serve as `nav2::ServiceServer`, and raytrace with `nav2_util`'s `raytraceLine` | The semantic layer and the slow escape behavior use them |
| Nav2's costmaps answer `GetCosts`, for several poses, instead of `GetCost` | `GetPosesAroundTable` asks it for one |
| Nav2's BT nodes, and their `BtActionNode` base, take a `nav2::LifecycleNode` from the blackboard | `thorp_bt_cpp` doesn't use `nav2_behavior_tree`: its action client nodes derive from `RosActionNode`, Noetic's base ported to ROS 2, on the server's `rclcpp::Node`, and `NavigateToPose`, `NavigateThroughPoses`, `FollowPath` and `BackUp` are Thorp's, with Nav2's IDs, action names and `error_code_id`, now also `GOAL_REJECTED` when a server refuses the goal; poses written in trees take Thorp's short forms, `x;y;yaw;frame` |
| `tinyxml2_vendor` is deprecated | `thorp_bt_cpp` finds the system's `tinyxml2` |
| `nav2_behavior_tree`'s CMake config finds `nav2_common`, which its package doesn't depend on | `thorp_bt_cpp` and `thorp_nav2_plugins` depend on it |
| rviz runs on Qt6 | `thorp_rviz_plugins` too |
| `pcl_conversions` exports a CMake target instead of include directories | `thorp_perception` links it |
| `ament_index_cpp` returns `std::filesystem::path`s; the string versions are deprecated | Paths |
| CMake 4 warns of compatibility below 3.10, and has no FindBoost module | `cmake_minimum_required(VERSION 3.10)`; packages that find Boost, or `geometric_shapes`, which finds it, set CMP0167 to use Boost's own config; `thorp_toolkit` links Boost privately, as only its sources use it, and doesn't export it |
| Gazebo Jetty's Python modules have no version: `gz.msgs`, `gz.transport` | `tray_objects` and `fill_tray` import them so |
| Gazebo Jetty is gz-sim 10 | `thorp_gazebo.launch.py` runs `gz sim --force-version 10` |
| SDF 1.12 names a sensor's frame with `<frame_id>`; Jetty still reads `<gz_frame_id>`, deprecated, and the parser warns of it | Sensors use `<frame_id>` |
| The controller spawner gives the controllers the parameter files of its own node, if it has any, instead of the controller manager's | It gets `controllers.yaml` with `--param-file` |
| ros2_control limits joint commands to the URDF's joint limits | The gripper and the cannon, commanded by steps, move at their joints' velocity limits; `gz_ros2_control` logs, throttled, an error for each clamped command |
| `tf2_ros`' listener and broadcasters take node interfaces; the constructors taking a node are deprecated | `thorp_toolkit` passes them its node's interfaces |
| Nav2 publishes and takes stamped velocity commands by default | `nav2.yaml` and `velocity_smoother.yaml` set `enable_stamped_cmd_vel: false` for every node, as `twist_mux`, the bridge and the base take unstamped ones |
| The rotation shim controller configures its primary controller in its own namespace, `primary_controller`, which takes the `plugin` parameter | MPPI's parameters are under `MPPI.primary_controller` |
| MPPI's motion models are plugins | `diff_drive` is `mppi::DiffDriveMotionModel` |
| The BT navigator takes `error_code_name_prefixes`, and refuses to start with `error_code_names` | The prefixes of the same two codes, `compute_path` and `follow_path` |
| `RemovePassedGoals` takes and updates the waypoints' statuses | `navigate_through_poses.xml` passes them the navigator's `waypoint_statuses` |
| Regulated pure pursuit's `desired_linear_vel` is deprecated | `max_linear_vel` |
| The rotation shim, Smac and graceful controllers aren't pulled in by other packages | `thorp_navigation` depends on them |
| xacro 2.1 resolves macro arguments in the caller's scope | No change in Thorp's model: every argument that names a parameter of the macro it calls would be a recursive definition in the macro's scope, so Jazzy's xacro resolved them in the caller's too. Not diffed against a Jazzy URDF, as none is left to compare with |

## Open

- AMCL's estimate drifts from Gazebo's ground truth on the playground, half a metre after crossing it; the apps
  localize with ground truth. Not measured on Jazzy.

## Known issues

- Shutting down a simulation with navigation takes 15 s: on shutdown, the planner and controller servers deactivate
  their costmaps, which wait for their update threads, sleeping on simulated time that stopped with Gazebo; launch
  kills them. Without simulated time they exit in under a second.
- `ros_gz_bridge`'s `parameter_bridge` crashes (SIGSEGV) when shutting down.
- `gz_ros2_control` logs, throttled, an error for every gripper and cannon command clamped to the joints' velocity
  limits.

- The range sensor layer's fix that clears the cone on readings above the maximum range lives on the user's
  `navigation2` fork, built from source on Jazzy. Lyrical builds without it, on Nav2's binary `nav2_costmap_2d`, until
  the fix is rebased onto Nav2's Lyrical release or merged upstream.
