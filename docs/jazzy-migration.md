# Thorp on ROS 2 Jazzy

The `jazzy` branch is the ROS 2 port of Thorp. The `noetic` branch keeps the ROS 1 version.

## Rules

- Target: ROS 2 Jazzy and Gazebo Harmonic, native ROS 2 only. No `ros1_bridge`, no mixed ROS 1 / ROS 2 runtime.
- Migrate in small blocks. Each block builds and runs on Jazzy on its own.
- Simulation first. Real hardware (Kobuki, arm servos, boards) is out of scope for now, but frame names, joint names
  and topics stay compatible with the real robot, so it can come back later.
- A package that is not migrated yet has a `COLCON_IGNORE` file; colcon and rosdep skip it. Migrating a package means
  porting it completely and deleting that file. Its ROS 1 code stays in place until then, for reference.
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
| 3 | Gazebo Harmonic: spawn Thorp, diff drive, joint states, Kinect, Xtion, sonars and IR sensors | next |
| 4 | Arm in simulation: `ros2_control`, trajectory and gripper controllers | |
| 5 | `thorp_msgs`, `thorp_toolkit` | |
| 6 | Navigation: Nav2 configuration, semantic costmap layer, MBF-specific behaviors | |
| 7 | Manipulation: MoveIt 2 configuration, pick and place servers | |
| 8 | Perception | |
| 9 | Executive: behavior trees and apps | |

Block 3 onwards will be refined as we get there.

## Package status

| Package | Status |
|---------|--------|
| thorp_description | migrated |
| thorp_apps, thorp_boards, thorp_bringup, thorp_bt_cpp, thorp_cannon, thorp_costmap_layers, thorp_exploration, thorp_manipulation, thorp_mbf_plugins, thorp_moveit_config, thorp_msgs, thorp_navigation, thorp_perception, thorp_rviz_plugins, thorp_simulation, thorp_smach, thorp_toolkit | ROS 1 (ignored) |
