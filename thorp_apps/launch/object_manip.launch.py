"""
Thorp's object manipulation app:
Pickup and place tabletop objects, as the user drags and drops them on RViz.

Requirements:
- Perception and manipulation
- User commands from RViz's User Commands panel: start, stop, reset, exit, clear gripper and fold arm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = LaunchConfiguration('simulator')
    includes = PathJoinSubstitution([FindPackageShare('thorp_apps'), 'launch', 'includes'])
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # App-specific parameters
        DeclareLaunchArgument('object_type', default_value='fixed', choices=['fixed', 'cubes', 'random']),

        # The exit command ends the tree, and with it the whole app
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'object_manip',
                              'simulator': simulator,
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'),
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'tabletop_manip.launch.py']),
            launch_arguments={'simulation': PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"]),
                              'object_type': LaunchConfiguration('object_type'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
    ])
