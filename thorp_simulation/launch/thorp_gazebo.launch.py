"""
Thorp simulation on Gazebo Harmonic:
- Gazebo, with the given world
- Thorp model, spawned from robot_description
- robot state publisher
- bridge between Gazebo and ROS topics
- point clouds and laser scans from the RGBD cameras
- velocity commands multiplexer
- arm, gripper and cannon controllers, and the cannon controller node
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable,
                            Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions.gzserver import GazeboRosPaths


def shutdown_unless_shutting_down(event, context):
    """ Shut everything down when Gazebo exits, as when closing its window; a second shutdown makes launch fail """
    return None if context.is_shutdown else [Shutdown(reason='Gazebo exited')]


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')

    sim_share = FindPackageShare('thorp_simulation')
    xacro_file = PathJoinSubstitution([FindPackageShare('thorp_description'), 'urdf', 'thorp.urdf.xacro'])
    controllers_file = PathJoinSubstitution([sim_share, 'param', 'controllers.yaml'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' simulation:=true',
                                                ' ros2_control_params:=', controllers_file]), value_type=str)
    sim_time = {'use_sim_time': True}

    # The environment ros_gz_sim's gz_sim.launch.py sets: models and plugins paths exported by packages, as Thorp's
    # meshes and cannon system, and libraries as plugins
    model_paths, plugin_paths = GazeboRosPaths.get_paths()
    gz_env = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join([os.environ.get('GZ_SIM_RESOURCE_PATH', ''), model_paths]),
              'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                                                            os.environ.get('LD_LIBRARY_PATH', ''), plugin_paths])}

    def gz_sim(server_only, condition):
        # -r: start running; -s: server only, with headless rendering for the sensors
        args = ['-r', '-s', '--headless-rendering'] if server_only else ['-r']
        # Not through gz_sim.launch.py, as its shell doesn't pass launch's signals on to Gazebo, left running when
        # launch shuts down other than with Ctrl-C
        return ExecuteProcess(cmd=['gz', 'sim'] + args + [world_file, '--force-version', '8'], name='gazebo',
                              output='screen', additional_env=gz_env, on_exit=shutdown_unless_shutting_down,
                              condition=condition)

    def point_cloud(camera):
        # Registered point cloud on the RGB optical frame, as produced by the real cameras' drivers
        return ComposableNode(package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzrgbNode',
                              name='points_xyzrgb', namespace=camera, parameters=[sim_time],
                              remappings=[('rgb/image_rect_color', 'rgb/image_raw'),
                                          ('depth_registered/image_rect', 'depth_registered/image_raw'),
                                          ('points', 'depth_registered/points')])

    bringup_share = FindPackageShare('thorp_bringup')

    return LaunchDescription([
        # To use a simulated world other than the default 'playground', provide either world_name
        # (without path nor extension), or the path to a world file. Optionally, provide also an initial pose
        DeclareLaunchArgument('world_name', default_value='playground',
                              description='World to load from thorp_simulation/worlds/gazebo'),
        DeclareLaunchArgument('world_file',
                              default_value=[PathJoinSubstitution([sim_share, 'worlds', 'gazebo', world_name]),
                                             '.world'],
                              description='Full path to the world file; overrides world_name'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),

        # Gazebo transport on loopback, as the simulation is local; on other interfaces, as a VPN's, its discovery
        # sometimes misses the bridge's subscription to the clock
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        gz_sim(False, IfCondition(gui)),
        gz_sim(True, UnlessCondition(gui)),

        # Gazebo publishes the clock on every simulation step (1 kHz); as Noetic's gazebo_ros, publish it at 100 Hz,
        # what sim time nodes can process cheaply
        Node(package='topic_tools', executable='throttle', name='clock_throttle', output='screen',
             arguments=['messages', '/clock_raw', '100.0', '/clock']),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'thorp', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('initial_pose_x'),
                        '-y', LaunchConfiguration('initial_pose_y'),
                        '-Y', LaunchConfiguration('initial_pose_a')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': PathJoinSubstitution([sim_share, 'param', 'gz_bridge.yaml'])}, sim_time]),

        # Objects resting on the tray get fixed to Thorp, so they don't join its contacts solving
        Node(package='thorp_simulation', executable='tray_objects.py', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('thorp_description'), 'config', 'tray.yaml']),
                         sim_time]),

        # Controllers run on Gazebo's controller manager, available once Thorp is spawned
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                        'cannon_joint_controller',
                        '--controller-manager-timeout', '60'],
             parameters=[sim_time]),

        # Arm support nodes, as on real robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('thorp_manipulation'), 'launch',
                                                                'includes', 'arm.launch.py'])),
            launch_arguments={'simulation': 'true'}.items()),

        # Same for the cannon: on simulation it talks with the cannon joint controller and firing system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('thorp_cannon'), 'launch',
                                                                'cannon.launch.py'])),
            launch_arguments={'simulation': 'true'}.items()),

        ComposableNodeContainer(package='rclcpp_components', executable='component_container',
                                name='cameras_container', namespace='', parameters=[sim_time],
                                composable_node_descriptions=[
                                    point_cloud('kinect'), point_cloud('xtion'),
                                    # Fake laser from Kinect (2D slice) and Xtion (3D projection)
                                    ComposableNode(package='depthimage_to_laserscan',
                                                   plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                                                   name='depthimage_to_laserscan', namespace='kinect',
                                                   parameters=[PathJoinSubstitution([bringup_share, 'param', 'kinect',
                                                                                     'depthimage_to_laserscan.yaml']),
                                                               sim_time],
                                                   remappings=[('depth', 'depth_registered/image_raw'),
                                                               ('depth_camera_info', 'depth_registered/camera_info')]),
                                    ComposableNode(package='pointcloud_to_laserscan',
                                                   plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                                                   name='pointcloud_to_laserscan', namespace='xtion',
                                                   parameters=[PathJoinSubstitution([bringup_share, 'param', 'xtion',
                                                                                     'pointcloud_to_laserscan.yaml']),
                                                               sim_time],
                                                   remappings=[('cloud_in', 'depth_registered/points')])]),

        # Velocity commands multiplexer
        Node(package='twist_mux', executable='twist_mux', name='cmd_vel_mux', output='screen',
             parameters=[PathJoinSubstitution([bringup_share, 'param', 'cmd_vel_mux.yaml']), sim_time],
             remappings=[('cmd_vel_out', '/mobile_base/commands/velocity')]),
    ])
