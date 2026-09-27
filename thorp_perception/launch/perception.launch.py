"""
Thorp perception:
- tables and tabletop objects detection on the Xtion point cloud
- Xtion field of view clearance check
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Not renamed, as launch renames all the nodes in the process, also PlanningSceneInterface's own node
        Node(package='thorp_perception', executable='object_detection', output='screen', respawn=True,
             parameters=[PathJoinSubstitution([FindPackageShare('thorp_perception'), 'config',
                                               'object_detection.yaml']), sim_time],
             remappings=[('cloud', '/xtion/depth_registered/points')]),

        Node(package='thorp_perception', executable='xtion_fov_analyzer.py', name='xtion_fov_analyzer',
             output='screen', respawn=True, parameters=[sim_time]),
    ])
