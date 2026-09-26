"""
Arm support nodes:
- fake state for the gripper joints no servo provides
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {'use_sim_time': LaunchConfiguration('simulation')}

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),

        Node(package='thorp_manipulation', executable='fake_joint_pub.py', name='fake_joint_pub',
             output='screen', respawn=True, parameters=[sim_time]),
    ])
