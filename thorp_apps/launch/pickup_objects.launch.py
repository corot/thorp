"""
Thorp's pickup reachable objects app:
Pickup all tabletop objects at hand from the table in front, and place them on the tray.

Requirements:
- Perception and manipulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = LaunchConfiguration('simulator')
    simulation = PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"])
    sim_time = {'use_sim_time': simulation}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # App-specific parameters
        DeclareLaunchArgument('object_type', default_value='fixed', choices=['fixed', 'cubes', 'rows', 'random']),

        # Components common for all apps
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_apps'), 'launch', 'includes', 'apps_common.launch.py']),
            launch_arguments={'app_name': 'pickup_objects',
                              'simulator': simulator,
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay')}.items()),

        # As we are not running navigation, provide a static global reference frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='fake_global_reference',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'], parameters=[sim_time]),

        # Spawn a table with some tabletop objects; the executive waits for it to finish
        Node(package='thorp_simulation', executable='spawn_gazebo_models.py', name='objects_spawner', output='screen',
             arguments=[['playground_', LaunchConfiguration('object_type')]], parameters=[sim_time],
             condition=IfCondition(simulation)),

        # Perception and manipulation
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_perception'), 'launch', 'perception.launch.py']),
            launch_arguments={'use_sim_time': simulation}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_manipulation'), 'launch', 'manipulation.launch.py']),
            launch_arguments={'simulation': simulation, 'use_sim_time': simulation}.items()),

        # RViz with MoveIt's motion planning panel, to see the state of move_group and the planning scene
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_moveit_config'), 'launch', 'moveit_rviz.launch.py']),
            launch_arguments={'simulation': simulation, 'use_sim_time': simulation}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
