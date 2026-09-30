"""
Thorp exploration planner, for the given camera's field of view
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='kinect', description='kinect or xtion'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='thorp_exploration', executable='exploration_planner.py', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('thorp_exploration'), 'param', 'exploration.yaml']),
                         {'camera': LaunchConfiguration('camera'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
