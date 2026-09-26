"""
Cannon controller, tilting and firing the cannon on command
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),

        Node(package='thorp_cannon', executable='cannon_ctrl.py', name='cannon_ctrl', output='screen',
             respawn=True, parameters=[{'use_sim_time': LaunchConfiguration('simulation')}]),
    ])
