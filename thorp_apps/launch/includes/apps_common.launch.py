"""
Nodes common to all apps:
- Thorp robot, simulated on Gazebo; the real robot's bringup is not ported yet
- executive: the app's behavior tree (bt), or bt_server offering all trees as capabilities (llm)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = LaunchConfiguration('simulator')
    executive = LaunchConfiguration('executive')
    simulation = PythonExpression(["'false' if '", simulator, "' == 'none' else 'true'"])
    # Into whichever node runs the app: bt_runner, or bt_server for the LLM executive
    apps_config = PathJoinSubstitution([FindPackageShare('thorp_apps'), 'param', 'apps_config.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('app_name'),
        DeclareLaunchArgument('simulator', default_value='gazebo', choices=['gazebo']),
        DeclareLaunchArgument('world_name', default_value='playground'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
        DeclareLaunchArgument('executive', default_value='bt', choices=['bt', 'llm']),
        DeclareLaunchArgument('viz_executive', default_value='false',
                              description='Publish the tree for Groot2, to a log file and to ~/bt_status'),
        DeclareLaunchArgument('start_delay', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value=simulation),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_simulation'), 'launch', ['thorp_', simulator, '.launch.py']]),
            launch_arguments={'world_name': LaunchConfiguration('world_name'),
                              'gui': LaunchConfiguration('gui'),
                              'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                              'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                              'initial_pose_a': LaunchConfiguration('initial_pose_a')}.items()),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_bt_cpp'), 'launch', 'bt_runner.launch.py']),
            launch_arguments={'app_name': LaunchConfiguration('app_name'),
                              'params_file': apps_config,
                              'start_delay': LaunchConfiguration('start_delay'),
                              'publish_bt': LaunchConfiguration('viz_executive'),
                              'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(PythonExpression(["'", executive, "' == 'bt'"]))),

        # Everything the app needs, but with no tree running: bt_server sits waiting instead, offering each tree
        # under bt/ as a capability over the RunSubtree action, for an LLM agent or the capability tests to call
        # one at a time. Use this rather than 'bt', so bt_runner isn't also ticking a whole app
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('thorp_bt_cpp'), 'launch', 'bt_server.launch.py']),
            launch_arguments={'params_file': apps_config,
                              'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(PythonExpression(["'", executive, "' == 'llm'"]))),
    ])
