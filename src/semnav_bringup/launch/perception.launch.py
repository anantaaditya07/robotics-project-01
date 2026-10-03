"""Perception bringup: yolo_onnx_node (architecture 7.1).

Parameters come from config/perception_params.yaml. The Fast DDS profile (D-19) is exported so
camera images arrive over shared memory.

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

    # D-19: Fast DDS profile with a large shared-memory segment so camera images are not lost.
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(bringup_dir, 'config', 'fastdds_profile.xml'))

    yolo = Node(
        package='semnav_perception',
        executable='yolo_onnx_node',
        name='yolo_onnx_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription(declare_args + [fastdds_profile, yolo])
