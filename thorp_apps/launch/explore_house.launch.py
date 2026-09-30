"""
Thorp's explore house app:
Explore the whole house, with the camera seeing all of it, room by room or all at once, as the exploration_method
parameter selects.

Requirements:
- Navigation
- Exploration planner
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
        DeclareLaunchArgument('world_name', default_value='fun_house'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl, static or gazebo'),
        DeclareLaunchArgument('initial_pose_x', default_value='8.5'),
        DeclareLaunchArgument('initial_pose_y', default_value='6.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
        # App-specific parameters: the camera whose field of view the exploration covers the house with
        DeclareLaunchArgument('camera', default_value='kinect', choices=['kinect', 'xtion']),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'explore_house',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'exploration.rviz',
                              'simulator': simulator,
                              'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'navigation.launch.py']),
            launch_arguments={'simulation': simulation,
                              'world_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_exploration'), 'launch', 'exploration.launch.py']),
            launch_arguments={'camera': LaunchConfiguration('camera'), 'use_sim_time': simulation}.items()),
    ])
