"""Localization bringup (Mode A): map_server + AMCL on a saved map.

Adapted from nav2_bringup's localization_launch.py (Humble 1.1.20). Plain Nodes only; the stock
composition/container branch is dropped. map_server and amcl are managed by
lifecycle_manager_localization and read their parameters from config/nav2_params.yaml.

AMCL needs an initial pose: publish one on /initialpose (RViz "2D Pose Estimate" or a script).
The saved map's frame equals the Gazebo world frame (slam_toolbox starts at the odom origin and
the diff-drive odometry is world-sourced), so the spawn pose (-2.0, -0.5, 0) is correct after a
fresh sim launch.

Usage (Mode A, with sim.launch.py running):
    ros2 launch semnav_bringup localization.launch.py
    ros2 launch semnav_bringup navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file},
            convert_types=True),
        allow_substs=True)

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'semnav_world.yaml'),
            description='Full path to the map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file (map_server and amcl sections)'),
        DeclareLaunchArgument(
            'autostart', default_value='true', description='Automatically start the lifecycle'),
        DeclareLaunchArgument(
            'use_respawn', default_value='false', description='Respawn nodes that crash'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ]

    return LaunchDescription(declare_args + [stdout_linebuf_envvar] + nodes)
