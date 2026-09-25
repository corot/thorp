"""
Thorp simulation on Gazebo Harmonic:
- Gazebo, with the given world
- Thorp model, spawned from robot_description
- robot state publisher
- bridge between Gazebo and ROS topics
- point clouds from the RGBD cameras
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')

    sim_share = FindPackageShare('thorp_simulation')
    xacro_file = PathJoinSubstitution([FindPackageShare('thorp_description'), 'urdf', 'thorp.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' simulation:=true']), value_type=str)
    sim_time = {'use_sim_time': True}

    def gz_sim(server_only, condition):
        # -r: start running; -s: server only, with headless rendering for the sensors
        args = '-r -s --headless-rendering ' if server_only else '-r '
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
            launch_arguments={'gz_args': [args, world_file], 'on_exit_shutdown': 'true'}.items(),
            condition=condition)

    def point_cloud(camera):
        # Registered point cloud on the RGB optical frame, as produced by the real cameras' drivers
        return ComposableNode(package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzrgbNode',
                              name='points_xyzrgb', namespace=camera, parameters=[sim_time],
                              remappings=[('rgb/image_rect_color', 'rgb/image_raw'),
                                          ('depth_registered/image_rect', 'depth_registered/image_raw'),
                                          ('points', 'depth_registered/points')])

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

        gz_sim(False, IfCondition(gui)),
        gz_sim(True, UnlessCondition(gui)),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'thorp', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('initial_pose_x'),
                        '-y', LaunchConfiguration('initial_pose_y'),
                        '-Y', LaunchConfiguration('initial_pose_a')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': PathJoinSubstitution([sim_share, 'param', 'gz_bridge.yaml'])}, sim_time]),

        ComposableNodeContainer(package='rclcpp_components', executable='component_container',
                                name='cameras_container', namespace='', parameters=[sim_time],
                                composable_node_descriptions=[point_cloud('kinect'), point_cloud('xtion')]),
    ])
