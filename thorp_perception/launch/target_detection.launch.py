"""
Thorp target detection, for the cat hunter:
- YOLO detects and tracks objects on the Kinect images, and locates them with its depth images
- the target tracker picks the target among them
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('yolo_model', default_value=os.path.expanduser('~/.cache/thorp/yolo11m.pt'),
                              description="YOLO weights; Ultralytics' own models get downloaded there if missing"),
        DeclareLaunchArgument('yolo_device', default_value='cuda:0', description='cuda:N or cpu'),

        # Scoped, as YOLO's arguments, as namespace, would collide with other launch files' ones
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py']),
                launch_arguments={'namespace': 'yolo',
                                  'model': LaunchConfiguration('yolo_model'),
                                  'device': LaunchConfiguration('yolo_device'),
                                  'threshold': '0.5',
                                  'use_tracking': 'True',
                                  'use_3d': 'True',
                                  'use_debug': 'True',
                                  'input_image_topic': '/kinect/rgb/image_raw',
                                  'input_depth_topic': '/kinect/depth_registered/image_raw',
                                  'input_depth_info_topic': '/kinect/depth_registered/camera_info',
                                  # a robot frame, fixed to the camera, as YOLO uses the latest transform
                                  'target_frame': 'base_footprint',
                                  # Gazebo's depth images are in meters; OpenNI's, in millimeters
                                  'depth_image_units_divisor': IfElseSubstitution(use_sim_time, '1', '1000')}.items()),
        ]),

        Node(package='thorp_perception', executable='target_tracker.py', name='target_tracker', output='screen',
             respawn=True, parameters=[{'use_sim_time': use_sim_time}]),
    ])
