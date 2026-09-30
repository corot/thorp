"""
Thorp's object gatherer app:
Explore the whole house looking for tables, and pick all the objects on each one onto the tray.

Requirements:
- Navigation
- Exploration planner
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
    includes = PathJoinSubstitution([FindPackageShare('thorp_apps'), 'launch', 'includes'])
    initial_pose = {f'initial_pose_{c}': LaunchConfiguration(f'initial_pose_{c}') for c in 'xya'}
    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='fun_house'),
        DeclareLaunchArgument('localization', default_value='gazebo', description='amcl, static or gazebo'),
        DeclareLaunchArgument('initial_pose_x', default_value='8.5'),
        DeclareLaunchArgument('initial_pose_y', default_value='6.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'object_gatherer',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'gathering.rviz',
                              'simulator': simulator,
                              'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'navigation.launch.py']),
            launch_arguments={'simulation': simulation,
                              'world_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'), **initial_pose}.items()),
        # The tables are found with the Xtion, so the exploration covers the house with its field of view
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_exploration'), 'launch', 'exploration.launch.py']),
            launch_arguments={'camera': 'xtion', 'use_sim_time': simulation}.items()),

        # Random tables with random objects, in open spaces of the house; the executive waits for them
        Node(package='thorp_simulation', executable='spawn_gazebo_models.py', name='objects_spawner', output='screen',
             arguments=['fun_house_objects'], parameters=[{'use_sim_time': simulation}],
             condition=IfCondition(simulation)),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_perception'), 'launch', 'perception.launch.py']),
            launch_arguments={'use_sim_time': simulation}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_manipulation'), 'launch', 'manipulation.launch.py']),
            launch_arguments={'simulation': simulation, 'use_sim_time': simulation}.items()),
    ])
