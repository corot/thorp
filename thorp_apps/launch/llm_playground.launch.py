"""
Thorp's LLM playground:
A fixed, repeatable scene for exercising capabilities one at a time, through bt_server's RunSubtree action: the LLM
agent driving the robot, or the capability test suite.

A table at (0.45, 0) with a fixed sample of objects on it, the robot half a meter short of it, facing it, and a still
cat 1.5 m to the robot's left, with the cannon's rocket. Same table, objects and cat every run; the test suite puts
the models back where they started between tests.

Requirements:
- Navigation
- Perception and manipulation
- Target detection
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
        # No tree of its own: bt_server offers them all
        DeclareLaunchArgument('executive', default_value='llm', choices=['llm']),
        DeclareLaunchArgument('viz_executive', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('localization', default_value='gazebo', description='amcl, static or gazebo'),
        # Far enough to approach the table, and close enough to detect it
        DeclareLaunchArgument('initial_pose_x', default_value='-0.5'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
        DeclareLaunchArgument('object_type', default_value='fixed', choices=['fixed', 'cubes', 'rows', 'random']),

        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'apps_common.launch.py']),
            launch_arguments={'app_name': 'llm_playground',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'bt_server.rviz',
                              'simulator': simulator,
                              'world_name': 'playground',
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'tabletop_manip.launch.py']),
            launch_arguments={'simulation': simulation,
                              'object_type': LaunchConfiguration('object_type'),
                              'static_map': 'false'}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([includes, 'navigation.launch.py']),
            launch_arguments={'simulation': simulation,
                              'world_name': 'playground',
                              'localization': LaunchConfiguration('localization'),
                              **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_perception'), 'launch', 'target_detection.launch.py']),
            launch_arguments={'use_sim_time': simulation}.items()),

        # The cat, and the cannon's rocket
        Node(package='thorp_simulation', executable='spawn_gazebo_models.py', name='cat_spawner', output='screen',
             arguments=['playground_cat'], parameters=[{'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cats_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([FindPackageShare('thorp_simulation'), 'param',
                                                               'cats_bridge.yaml']),
                          'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
        # The cat stays still; the controller only sends it off the map once toppled
        Node(package='thorp_simulation', executable='cats_controller.py', name='cats_controller', output='screen',
             respawn=True, parameters=[{'prowling_speed': 0.0,
                                        'outside_x': 5.0,
                                        'outside_y': 5.0,
                                        'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
    ])
