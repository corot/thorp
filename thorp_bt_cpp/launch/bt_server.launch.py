"""
Offer every behavior tree under bt/ as a capability, through the bt_server/run_subtree action
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tray_params = PathJoinSubstitution([FindPackageShare('thorp_description'), 'config', 'tray.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='App parameters'),
        DeclareLaunchArgument('tick_rate', default_value='10.0'),
        DeclareLaunchArgument('bt_dir', default_value=PathJoinSubstitution([FindPackageShare('thorp_bt_cpp'), 'bt'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(package='thorp_bt_cpp', executable='bt_server_node', output='screen',
             parameters=[tray_params, LaunchConfiguration('params_file'),
                         {'tick_rate': LaunchConfiguration('tick_rate'),
                          'bt_dir': LaunchConfiguration('bt_dir'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
