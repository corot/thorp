"""
RViz with the MoveIt motion planning plugin
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
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', str(config.package_path / 'config' / 'moveit.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, config.joint_limits,
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false',
                              description='Use the ideal camera poses instead of the calibrated ones'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
