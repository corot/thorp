"""
Thorp's cat aiming app, to see how the cat hunter follows and aims:
Still cats around the house; the robot stays put until it sees one, then approaches it and keeps aiming the cannon at
it while following it, without firing. Move a cat into the robot's view with the Gazebo GUI to start.

Requirements:
- Navigation
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
            launch_arguments={'app_name': 'aim_at_cats',
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': 'hunting.rviz',
                              'simulator': simulator,
                              'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'),
                              'executive': LaunchConfiguration('executive'),
                              'viz_executive': LaunchConfiguration('viz_executive'),
                              'start_delay': LaunchConfiguration('start_delay'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'launch', 'navigation.launch.py']),
            launch_arguments={'use_sim_time': simulation,
                              'map_name': LaunchConfiguration('world_name'),
                              'localization': LaunchConfiguration('localization'), **initial_pose}.items()),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_perception'), 'launch', 'target_detection.launch.py']),
            launch_arguments={'use_sim_time': simulation}.items()),

        # The cat hunter's cats, randomly around the house, and the cannon's rocket
        Node(package='thorp_simulation', executable='spawn_gazebo_models.py', name='objects_spawner', output='screen',
             arguments=['cats'], parameters=[{'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='cats_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([FindPackageShare('thorp_simulation'), 'config',
                                                               'cats_bridge.yaml']),
                          'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
        # The cats stay still; the controller only sends them off the map once toppled
        Node(package='thorp_simulation', executable='cats_controller.py', name='cats_controller', output='screen',
             respawn=True, parameters=[{'prowling_speed': 0.1,
                                        'outside_x': 1.0,
                                        'outside_y': -5.0,
                                        'use_sim_time': simulation}],
             condition=IfCondition(simulation)),
    ])
