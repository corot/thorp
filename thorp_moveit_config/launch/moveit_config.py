"""
Thorp MoveIt configuration, shared by the launch files
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def thorp_moveit_config(simulation):
    """ MoveIt configuration for Thorp; simulation selects the ideal camera poses, as in thorp_description """
    xacro_file = os.path.join(get_package_share_directory('thorp_description'), 'urdf', 'thorp.urdf.xacro')
    return (MoveItConfigsBuilder('Thorp', package_name='thorp_moveit_config')
            .robot_description(file_path=xacro_file, mappings={'simulation': simulation})
            .robot_description_semantic(file_path='config/Thorp.srdf')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .joint_limits(file_path='config/joint_limits.yaml')
            .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .sensors_3d(file_path='config/sensors_3d.yaml')
            # robot_state_publisher publishes the description; this one lacks the ros2_control parameters Gazebo needs
            .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
            .to_moveit_configs())
