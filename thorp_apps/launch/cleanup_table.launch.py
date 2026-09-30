"""
Thorp's cleanup table app:
Pick all the objects on the table in front, from as few locations around it as possible, and place them on the tray.

Requirements:
- Perception and manipulation
- Navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = LaunchConfiguration('simulator')
    simulation = PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"])
    includes = PathJoinSubstitution([FindPackageShare('thorp_apps'), 'launch', 'includes'])
    initial_pose = {f'initial_pose_{c}': LaunchConfiguration(f'initial_pose_{c}') for c in 'xya'}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('localization', default_value='gazebo', description='amcl, static or gazebo'),
        DeclareLaunchArgument('initial_pose_x', default_value='-0.5'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
        # App-specific parameters: objects all over the table, not only within reach of the robot
        DeclareLaunchArgument('object_type', default_value='random', choices=['fixed', 'cubes', 'rows', 'random']),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'cleanup_table',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'cleanup_table.rviz',
                              'simulator': simulator,
                              'world_name': 'playground',
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'tabletop_manip.launch.py']),
            launch_arguments={'simulation': simulation,
                              'object_type': LaunchConfiguration('object_type'),
                              'static_map': 'false'}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'navigation.launch.py']),
            launch_arguments={'simulation': simulation,
                              'world_name': 'playground',
                              'localization': LaunchConfiguration('localization'),
                              **initial_pose}.items()),
    ])
