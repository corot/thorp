"""
Show Thorp's robot model on RViz, with sliders to move its joints.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    pkg_share = FindPackageShare('thorp_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'thorp.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' simulation:=', simulation]),
                                       value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false',
                              description='Use the ideal camera poses instead of the calibrated ones'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Move the joints with joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])],
             condition=IfCondition(rviz)),
    ])
