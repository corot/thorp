"""
Thorp MoveIt move group
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from moveit_config import thorp_moveit_config  # noqa: E402


def launch_setup(context):
    config = thorp_moveit_config(LaunchConfiguration('simulation').perform(context))
    return [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', respawn=True,
             # The controller manager logs its controllers list on every lookup, several times per trajectory
             arguments=['--ros-args', '--log-level',
                        'move_group.moveit.moveit.plugins.simple_controller_manager:=warn'],
             parameters=[config.to_dict(),
                         {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'allow_trajectory_execution': True,
                          # executes MoveIt Task Constructor solutions, as Thorp's pick and place tasks
                          'capabilities': 'move_group/ExecuteTaskSolutionCapability',
                          'max_safe_path_cost': 1.0,
                          'jiggle_fraction': 0.05,
                          'publish_monitored_planning_scene': True,
                          'octomap_resolution': 0.015,
                          'max_range': 5.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false',
                              description='Use the ideal camera poses instead of the calibrated ones'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
