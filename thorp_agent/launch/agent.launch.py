"""
The LLM agent, for one-shot questions (ask:=...): launch gives a node no stdin, so talk to it with ros2 run.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false', description='Print the goals instead of sending them'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('ask', default_value='', description='One question, then exit'),

        Node(package='thorp_agent', executable='agent.py', output='screen',
             parameters=[{'dry_run': LaunchConfiguration('dry_run'),
                          'verbose': LaunchConfiguration('verbose'),
                          'ask': ParameterValue(LaunchConfiguration('ask'), value_type=str)}]),
    ])
