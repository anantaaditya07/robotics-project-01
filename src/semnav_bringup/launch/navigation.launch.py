"""Navigation bringup: the Nav2 navigation servers for SemNav (no localization, no smoother).

Adapted from nav2_bringup's navigation_launch.py (Humble 1.1.20). Plain Nodes only; the stock
composition/container branch is dropped. Started nodes, all managed by
lifecycle_manager_navigation: controller_server, smoother_server, planner_server,
behavior_server, bt_navigator, waypoint_follower.

Command velocity routing:
- D-01: no velocity_smoother. controller_server publishes /cmd_vel_nav; safety_gate_node is the
  only publisher of /cmd_vel.
- D-02: behavior_server (spin, backup, wait) is also remapped cmd_vel -> cmd_vel_nav so recoveries
  pass through the safety gate too.
- D-16: until safety_gate_node exists, the cmd_vel_relay arg (default true) runs
  `topic_tools relay /cmd_vel_nav /cmd_vel`. Set it false once the gate runs, otherwise /cmd_vel
  has two publishers.

Localization (map_server/AMCL or the map->odom transform) is NOT started here. It comes either
from slam_toolbox (mapping.launch.py) or from a separate localization launch.

Usage (with sim.launch.py and mapping.launch.py already running):
    ros2 launch semnav_bringup navigation.launch.py
Once safety_gate_node runs:
    ros2 launch semnav_bringup navigation.launch.py cmd_vel_relay:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    cmd_vel_relay = LaunchConfiguration('cmd_vel_relay')

    # No velocity_smoother (D-01).
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Same as stock: make tf topics relative so a namespace can be prepended.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Controller and behaviors both publish to cmd_vel_nav, the safety gate input (D-01, D-02).
    cmd_vel_remap = [('cmd_vel', 'cmd_vel_nav')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_args = [
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file for all launched nodes'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate the Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn a Nav2 node if it crashes'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument(
            'cmd_vel_relay', default_value='true',
            description='Interim relay /cmd_vel_nav -> /cmd_vel until safety_gate_node exists '
                        '(D-16); set false when the gate runs'),
    ]

    # D-20: CycloneDDS for every SemNav process (Fast DDS lost TF between Nav2 servers).
    dds_env = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI', 'file://' + os.path.join(bringup_dir, 'config', 'cyclonedds.xml')),
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    def nav2_node(package, executable, extra_remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + (extra_remappings or []))

    nav2_nodes = [
        nav2_node('nav2_controller', 'controller_server', cmd_vel_remap),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', cmd_vel_remap),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    # topic_tools relay CLI: relay <intopic> [outtopic] (D-16).
    relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel_nav', '/cmd_vel'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(cmd_vel_relay))

    env = [stdout_linebuf_envvar] + dds_env
    return LaunchDescription(declare_args + env + nav2_nodes + [
        lifecycle_manager,
        relay,
    ])
