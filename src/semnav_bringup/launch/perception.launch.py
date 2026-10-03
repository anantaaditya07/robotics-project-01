"""Perception bringup: yolo_onnx_node (7.1) and semantic_fusion_node (7.2).

Parameters come from config/perception_params.yaml. RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and
CYCLONEDDS_URI (config/cyclonedds.xml) are exported, like in every SemNav launch file (D-20).

semantic_fusion_node needs the map frame: run localization.launch.py (or mapping.launch.py).

Usage (with sim.launch.py running):
    ros2 launch semnav_bringup perception.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'perception_params.yaml'),
            description='Perception parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
    ]

    # D-20: CycloneDDS for every SemNav process (Fast DDS lost TF between Nav2 servers).
    dds_env = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI', 'file://' + os.path.join(bringup_dir, 'config', 'cyclonedds.xml')),
    ]

    yolo = Node(
        package='semnav_perception',
        executable='yolo_onnx_node',
        name='yolo_onnx_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    fusion = Node(
        package='semnav_perception',
        executable='semantic_fusion_node',
        name='semantic_fusion_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription(declare_args + dds_env + [yolo, fusion])
