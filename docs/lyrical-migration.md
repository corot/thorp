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
| 1 | Workspace: branch, this document, `thorp-lyrical.repos`, dependencies from Lyrical binaries | in progress |
| 2 | Build: every package compiles on Lyrical, with no new warnings | in progress |
| 3 | `thorp_description`: the same URDF as on Jazzy | |
| 4 | Simulation on Gazebo Jetty: spawn, sensors, controllers, cannon, tray objects | |
| 5 | Navigation on Nav2 1.5: localization, costmaps and their layers, controllers, behaviors | |
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
| `nav2_behavior_tree`'s CMake config finds `nav2_common`, which its package doesn't depend on | `thorp_bt_cpp` and `thorp_nav2_plugins` depend on it |
| rviz runs on Qt6 | `thorp_rviz_plugins` too |
| `pcl_conversions` exports a CMake target instead of include directories | `thorp_perception` links it |
| `ament_index_cpp` returns `std::filesystem::path`s; the string versions are deprecated | Paths |
| CMake 4 warns of compatibility below 3.10, and has no FindBoost module | `cmake_minimum_required(VERSION 3.10)`; packages that find Boost, or `geometric_shapes`, which finds it, set CMP0167 to use Boost's own config; `thorp_toolkit` links Boost privately, as only its sources use it, and doesn't export it |
| Gazebo Jetty's Python modules have no version: `gz.msgs`, `gz.transport` | `tray_objects` and `fill_tray` import them so |

## Open

- The range sensor layer's fix that clears the cone on readings above the maximum range lives on the user's
  `navigation2` fork, built from source on Jazzy. Lyrical builds without it, on Nav2's binary `nav2_costmap_2d`, until
  the fix is rebased onto Nav2's Lyrical release or merged upstream.
