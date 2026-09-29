"""
Thorp's patrol app:
Patrol between two points of the playground map.

Requirements:
- Navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = LaunchConfiguration('simulator')
    includes = PathJoinSubstitution([FindPackageShare('thorp_apps'), 'launch', 'includes'])
    initial_pose = {f'initial_pose_{c}': LaunchConfiguration(f'initial_pose_{c}') for c in 'xya'}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # The tree's poses are on the playground map
        DeclareLaunchArgument('world_name', default_value='playground'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl, static or gazebo'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'patrol_2_points',
                              'simulator': simulator,
                              'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'navigation.launch.py']),
            launch_arguments={'simulation': PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"]),
                              'world_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'),
                              'rviz': LaunchConfiguration('rviz'), **initial_pose}.items()),
    ])
