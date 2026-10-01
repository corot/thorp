"""
Run thorp_world's node, loading thorp_world_file if it exists. Not named world_file, as the simulator's launch
sets that, and included launch files share their arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('thorp_world_file', default_value='',
                              description='YAML file to load the world from on start, and to save it to'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(package='thorp_world', executable='world_node.py', name='world', output='screen',
             parameters=[{'world_file': LaunchConfiguration('thorp_world_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
