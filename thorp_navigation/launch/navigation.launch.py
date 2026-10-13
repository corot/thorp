"""
Thorp navigation:
- Nav2: planner, controller, smoother, behaviors and BT navigator, taking goals from RViz
- object following
- velocity smoothers for both, in front of their velocity multiplexer inputs
- velocity and travelled distance display
- geometric map
- localization: AMCL, static or Gazebo ground truth
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Velocity smoother: the source whose <source>/cmd_vel it smooths into <source>/cmd_vel_smoothed, a multiplexer input
SMOOTHERS = {'navigation_smoother': 'navigation', 'following_smoother': 'following'}


def behavior_tree(navigator):
    """ bt_navigator's tree for navigate_to_pose or navigate_through_poses """
    return os.path.join(get_package_share_directory('thorp_navigation'), 'behavior_trees', f'navigate_{navigator}.xml')


def launch_setup(context):
    localization = LaunchConfiguration('localization').perform(context)
    initial_pose = [float(LaunchConfiguration(f'initial_pose_{c}').perform(context)) for c in 'xya']

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    common_params = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]

    cmd_vel_remap = [('cmd_vel', '/navigation/cmd_vel')]
    smoother_params = [PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'config', 'velocity_smoother.yaml']),
                       {'use_sim_time': LaunchConfiguration('use_sim_time')}]

    navigation_nodes = ['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator',
                        'following_server', *SMOOTHERS]
    localization_nodes = ['map_server']
    if localization == 'amcl':
        localization_nodes.append('amcl')

    actions = [
        # ****************** Nav2 ******************
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             respawn=True, parameters=common_params, remappings=cmd_vel_remap),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             respawn=True, parameters=common_params),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen',
             respawn=True, parameters=common_params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             respawn=True, parameters=common_params, remappings=cmd_vel_remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             respawn=True, parameters=common_params + [{'default_nav_to_pose_bt_xml': behavior_tree('to_pose'),
                                                       'default_nav_through_poses_bt_xml':
                                                           behavior_tree('through_poses')}]),
        # Following goes through its own smoother to its own multiplexer input
        Node(package='opennav_following', executable='opennav_following', name='following_server', output='screen',
             respawn=True, parameters=common_params, remappings=[('cmd_vel', '/following/cmd_vel')]),
        *[Node(package='nav2_velocity_smoother', executable='velocity_smoother', name=name, output='screen',
               respawn=True, parameters=smoother_params,
               remappings=[('cmd_vel', f'/{source}/cmd_vel'), ('cmd_vel_smoothed', f'/{source}/cmd_vel_smoothed')])
          for name, source in SMOOTHERS.items()],
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                           'autostart': True, 'node_names': navigation_nodes}]),

        # ****************** Visual aids ******************
        Node(package='thorp_navigation', executable='show_velocity.py', name='show_velocity', output='screen',
             respawn=True, parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        # ****************** Geometric map server ******************
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', respawn=True,
             parameters=common_params + [{'yaml_filename': map_file}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                           'autostart': True, 'node_names': localization_nodes}]),
    ]

    if localization == 'amcl':
        actions.append(
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', respawn=True,
                 parameters=common_params + [{'set_initial_pose': True,
                                              'initial_pose.x': initial_pose[0],
                                              'initial_pose.y': initial_pose[1],
                                              'initial_pose.yaw': initial_pose[2]}]))
    elif localization == 'static':
        actions.append(
            Node(package='tf2_ros', executable='static_transform_publisher', name='static_localization',
                 output='screen', respawn=True,
                 arguments=['--x', str(initial_pose[0]), '--y', str(initial_pose[1]), '--yaw', str(initial_pose[2]),
                            '--frame-id', 'map', '--child-frame-id', 'odom']))
    elif localization == 'gazebo':
        actions.append(
            Node(package='thorp_simulation', executable='gazebo_ground_truth', name='gazebo_ground_truth',
                 output='screen', respawn=True, parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]))
    else:
        raise ValueError(f'Unknown localization {localization}; must be amcl, static or gazebo')

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl, static or gazebo'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'config',
                                                                  'nav2.yaml']),
                              description='Nav2 parameters file'),

        # Name of the map to use (without path nor extension) and initial position
        DeclareLaunchArgument('map_name', default_value='playground'),
        DeclareLaunchArgument('map_file',
                              default_value=[PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'maps',
                                                                   LaunchConfiguration('map_name')]), '.yaml']),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        OpaqueFunction(function=launch_setup),
    ])
