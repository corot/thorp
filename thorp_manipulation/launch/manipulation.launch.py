"""
Thorp manipulation:
- MoveIt move group
- pickup and place object action servers
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

moveit_launch_dir = os.path.join(get_package_share_directory('thorp_moveit_config'), 'launch')
sys.path.append(moveit_launch_dir)
from moveit_config import thorp_moveit_config  # noqa: E402


def launch_setup(context):
    simulation = LaunchConfiguration('simulation').perform(context)
    params_file = 'pick_and_place_gazebo.yaml' if simulation == 'true' else 'pick_and_place.yaml'
    config = thorp_moveit_config(simulation)
    return [
        IncludeLaunchDescription(os.path.join(moveit_launch_dir, 'move_group.launch.py'),
                                 launch_arguments={'simulation': simulation,
                                                   'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='thorp_manipulation', executable='manipulation_node', name='manipulation', output='screen',
             respawn=True,
             parameters=[config.to_dict(),
                         os.path.join(get_package_share_directory('thorp_manipulation'), 'param', params_file),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
