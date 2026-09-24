thorp_description
=================

Thorp's robot model: a TurtleBot 2 (Kobuki base, hexagon stacks and Kinect) with a TurtleBot arm, a tray, a cannon,
an Asus Xtion Pro manipulation camera, sonars and IR sensors.

View it on RViz, with sliders to move the joints:

```
ros2 launch thorp_description display.launch.py
```

`simulation:=true` uses the ideal camera poses instead of the calibrated ones.

Copied third-party files
------------------------

These files come from ROS 1 packages that have no ROS 2 Jazzy release. They are copies of the exact versions Thorp
used on Noetic, with package paths changed to `thorp_description`. All are BSD licensed.

| Files | Source | Other changes |
|-------|--------|---------------|
| `urdf/turtlebot/kobuki.urdf.xacro`, `urdf/turtlebot/common_properties.urdf.xacro`, `meshes/kobuki/` | `kobuki_description` from [corot/kobuki](https://github.com/corot/kobuki), `thorp` branch, commit `5d934bc` | Gazebo Classic include and `kobuki_sim` call removed |
| `urdf/turtlebot/hexagons.urdf.xacro`, `urdf/turtlebot/turtlebot_properties.urdf.xacro`, `meshes/turtlebot/` | `turtlebot_description` from [turtlebot/turtlebot](https://github.com/turtlebot/turtlebot), `melodic` branch, commit `bd830d8` | none |
| `urdf/turtlebot_arm/`, `meshes/turtlebot_arm/` | `turtlebot_arm_description` from [corot/turtlebot_arm](https://github.com/corot/turtlebot_arm), `thorp` branch, commit `d73b53a` | `M_PI` set to full precision (was 3.14159) |
| `meshes/sensors/max_sonar_ez4.dae` | `hector_sensors_description` from [tu-darmstadt-ros-pkg/hector_models](https://github.com/tu-darmstadt-ros-pkg/hector_models), `melodic-devel` branch, commit `ebc07c1` | none |

Legacy files
------------

- `urdf/thorp_gazebo.urdf.xacro`: Gazebo Classic sensors, plugins and transmissions. No longer included; it will be
  replaced by Gazebo Harmonic equivalents when migrating the simulation.
- `urdf/senz3d.urdf.xacro` and `urdf/thorp.urdf.xacro.senz3d`: old Senz3D camera variant, not maintained.
