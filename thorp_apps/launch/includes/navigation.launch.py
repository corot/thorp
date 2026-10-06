"""
Navigation for the apps that move around:
- Nav2 and localization, on the map of the world
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    initial_pose = {f'initial_pose_{c}': LaunchConfiguration(f'initial_pose_{c}') for c in 'xya'}
    return LaunchDescription([
        DeclareLaunchArgument('simulation'),
        DeclareLaunchArgument('world_name', description='Also the map name'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl, static or gazebo'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        # Smoothed velocities, as braking hard tilts the robot
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'launch', 'navigation.launch.py']),
            launch_arguments={'use_sim_time': simulation,
                              'smooth_velocity': 'true',
                              'map_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'), **initial_pose}.items()),
    ])
