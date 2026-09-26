"""
Thorp navigation:
- Nav2: planner, controller, smoother, behaviors and BT navigator, taking goals from RViz
- object following
- velocity and travelled distance display
- velocity smoother
- geometric map
- localization: AMCL, static or Gazebo ground truth
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    localization = LaunchConfiguration('localization').perform(context)
    smooth_velocity = LaunchConfiguration('smooth_velocity').perform(context) == 'true'
    initial_pose = [float(LaunchConfiguration(f'initial_pose_{c}').perform(context)) for c in 'xya']

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    common_params = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]

    # Velocity commands go to the multiplexer navigation input, through the velocity smoother if enabled
    nav_cmd_vel = '/vel_smoother/raw_cmd_vel' if smooth_velocity else '/cmd_vel_mux/input/navigation'
    cmd_vel_remap = [('cmd_vel', nav_cmd_vel)]

    navigation_nodes = ['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator',
                        'following_server']
    if smooth_velocity:
        navigation_nodes.append('velocity_smoother')
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
             respawn=True, parameters=common_params),
        # Following goes to its own multiplexer input, as Noetic's pose follower
        Node(package='opennav_following', executable='opennav_following', name='following_server', output='screen',
             respawn=True, parameters=common_params, remappings=[('cmd_vel', '/cmd_vel_mux/input/following')]),
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

    if smooth_velocity:
        actions.append(
            Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
                 output='screen', respawn=True, parameters=common_params,
                 remappings=[('cmd_vel', nav_cmd_vel), ('cmd_vel_smoothed', '/cmd_vel_mux/input/navigation')]))

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
        DeclareLaunchArgument('smooth_velocity', default_value='true',
                              description='Smooth navigation velocity commands'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([FindPackageShare('thorp_navigation'), 'param',
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
