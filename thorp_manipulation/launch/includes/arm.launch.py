"""
Arm support nodes:
- gripper command action server, taking openings in meters
- fake state for the gripper joints no servo provides
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {'use_sim_time': LaunchConfiguration('simulation')}

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),

        Node(package='thorp_manipulation', executable='gripper_controller.py', name='gripper_controller',
             output='screen', respawn=True,
             parameters=[PathJoinSubstitution([FindPackageShare('thorp_manipulation'), 'param',
                                               'gripper_controller.yaml']), sim_time]),
        Node(package='thorp_manipulation', executable='fake_joint_pub.py', name='fake_joint_pub',
             output='screen', respawn=True, parameters=[sim_time]),
    ])
