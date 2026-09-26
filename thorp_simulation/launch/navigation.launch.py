"""
Thorp simulated navigation:
- simulated robot on Gazebo
- navigation
- rviz view
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    initial_pose = {f'initial_pose_{c}': LaunchConfiguration(f'initial_pose_{c}') for c in 'xya'}

    return LaunchDescription([
        DeclareLaunchArgument('visualization', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl, static or gazebo'),
        DeclareLaunchArgument('world_name', default_value='playground'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        # ****************** Simulated Thorp ******************
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('thorp_simulation'), 'launch',
                                                                'thorp_gazebo.launch.py'])),
            launch_arguments={'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'), **initial_pose}.items()),

        # ****************** Navigation ******************
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'launch',
                                                                'navigation.launch.py'])),
            launch_arguments={'use_sim_time': 'true',
                              'smooth_velocity': 'false',
                              'map_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'), **initial_pose}.items()),

        # ****************** Visualization ******************
        Node(package='rviz2', executable='rviz2', name='rviz', output='screen', respawn=True,
             arguments=['-d', PathJoinSubstitution([FindPackageShare('thorp_bringup'), 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('visualization'))),
    ])
