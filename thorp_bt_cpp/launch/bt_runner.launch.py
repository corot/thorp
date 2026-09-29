"""
Run an app's behavior tree, bt/<app_name>.xml, whose root tree is named as the app
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def shutdown_on_exit(event, context):
    """ Shut everything down when the tree completes; not if already shutting down, as a second shutdown makes launch
    fail """
    return None if context.is_shutdown else [Shutdown(reason='App completed')]


def runner(context):
    """ The runner node, shutting everything down on exit if requested, as read when launched """
    app_name = LaunchConfiguration('app_name')
    tray_params = PathJoinSubstitution([FindPackageShare('thorp_description'), 'config', 'tray.yaml'])
    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown').perform(context) == 'true'
    return [
        # Named after the app with an argument, not with the node name
        Node(package='thorp_bt_cpp', executable='bt_runner_node', output='screen', arguments=[app_name],
             on_exit=shutdown_on_exit if on_exit_shutdown else None,
             parameters=[tray_params, LaunchConfiguration('params_file'),
                         {'app_name': app_name,
                          'start_delay': LaunchConfiguration('start_delay'),
                          'publish_bt': LaunchConfiguration('publish_bt'),
                          'bt_filepath': [PathJoinSubstitution([FindPackageShare('thorp_bt_cpp'), 'bt', app_name]),
                                          '.xml'],
                          'nodes_filepath': LaunchConfiguration('nodes_filepath'),
                          # The runner usually starts with the servers its tree uses, so it waits longer for them
                          'wait_for_service_timeout': 60000,
                          'use_sim_time': LaunchConfiguration('use_sim_time')}])
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('app_name'),
        DeclareLaunchArgument('params_file', description='App parameters'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('publish_bt', default_value='false',
                              description='Publish the tree for Groot2, to a log file and to ~/bt_status'),
        DeclareLaunchArgument('nodes_filepath', default_value='',
                              description='Write the node models there, for Groot2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('on_exit_shutdown', default_value='false',
                              description='Shut down the whole launch when the tree completes'),

        OpaqueFunction(function=runner),
    ])
