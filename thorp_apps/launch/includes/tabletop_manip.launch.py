"""
Nodes common to the apps manipulating objects on the table in front of Thorp, with no navigation:
- static global reference frame, unless navigation localizes the robot
- a table with some tabletop objects, on simulation
- perception and manipulation
- RViz, to see the state of move_group and interact with the app
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    sim_time = {'use_sim_time': simulation}
    return LaunchDescription([
        DeclareLaunchArgument('simulation'),
        DeclareLaunchArgument('object_type', description='Objects to spawn on simulation'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('static_map', default_value='true', description='Map fixed on the odometry origin'),

        # As we are not running navigation, provide a static global reference frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='fake_global_reference',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'], parameters=[sim_time],
             condition=IfCondition(LaunchConfiguration('static_map'))),

        # Spawn a table with some tabletop objects; the executive waits for it to finish
        Node(package='thorp_simulation', executable='spawn_gazebo_models.py', name='objects_spawner', output='screen',
             arguments=[['playground_', LaunchConfiguration('object_type')]], parameters=[sim_time],
             condition=IfCondition(simulation)),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_perception'), 'launch', 'perception.launch.py']),
            launch_arguments={'use_sim_time': simulation}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_manipulation'), 'launch', 'manipulation.launch.py']),
            launch_arguments={'simulation': simulation, 'use_sim_time': simulation}.items()),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_moveit_config'), 'launch', 'moveit_rviz.launch.py']),
            launch_arguments={'simulation': simulation, 'use_sim_time': simulation,
                              'rviz_config': PathJoinSubstitution([FindPackageShare('thorp_bringup'), 'rviz',
                                                                   'manipulation.rviz'])}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
