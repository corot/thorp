"""
Thorp's pickup reachable objects app:
Pickup all tabletop objects at hand from the table in front, and place them on the tray.

Requirements:
- Perception and manipulation
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
        DeclareLaunchArgument('object_type', default_value='fixed', choices=['fixed', 'cubes', 'rows', 'random']),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'pickup_objects',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'manipulation.rviz',
                              'simulator': simulator,
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay')}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'tabletop_manip.launch.py']),
            launch_arguments={'simulation': PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"]),
                              'object_type': LaunchConfiguration('object_type')}.items()),
    ])
