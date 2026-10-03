"""SemNav full system: one command starts everything (architecture section 12, Phase 7/8).

Starts, in this order:
  1. sim.launch.py           Gazebo + SemNav waffle + world (RViz optional, rviz_software_gl, D-17)
  2. localization.launch.py  map_server + AMCL on maps/semnav_world.yaml, initial pose set
                             automatically at the spawn pose (no /initialpose needed)
  3. perception.launch.py    yolo_onnx_node + semantic_fusion_node
  4. navigation.launch.py    Nav2 (semantic layer in both costmaps) + safety_gate_node, the only
                             /cmd_vel publisher (D-01, D-02)
Steps 2-4 start `stack_delay` seconds after the simulation so the robot is spawned first.

Prerequisite in the shell (D-20, see README): the tf2 underlay must be sourced
    source /opt/ros/humble/setup.bash
    source ~/semnav_underlay/install/setup.bash
    source install/setup.bash

Usage:
    ros2 launch semnav_bringup semnav.launch.py                      # Gazebo GUI + RViz
    ros2 launch semnav_bringup semnav.launch.py gui:=false rviz:=false   # headless
    ros2 launch semnav_bringup semnav.launch.py rviz_software_gl:=true   # Intel GL (D-17)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_software_gl = LaunchConfiguration('rviz_software_gl')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    nav2_params = LaunchConfiguration('nav2_params')
    perception_params = LaunchConfiguration('perception_params')
    stack_delay = LaunchConfiguration('stack_delay')

    declare_args = [
        DeclareLaunchArgument('gui', default_value='true', description='Start gzclient'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_software_gl', default_value='false',
            description='Render RViz with Mesa software GL (D-17)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup_dir, 'maps', 'semnav_world.yaml'),
            description='Saved map for map_server + AMCL'),
        DeclareLaunchArgument('x_pose', default_value='-2.0', description='Spawn / AMCL x (m)'),
        DeclareLaunchArgument('y_pose', default_value='-0.5', description='Spawn / AMCL y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='AMCL initial yaw (rad)'),
        DeclareLaunchArgument(
            'nav2_params', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'perception_params',
            default_value=os.path.join(bringup_dir, 'config', 'perception_params.yaml'),
            description='Perception parameter file'),
        DeclareLaunchArgument(
            'stack_delay', default_value='8.0',
            description='Seconds after the simulation before localization/perception/Nav2 start'),
    ]

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items())

    sim = include('sim.launch.py', {
        'gui': gui, 'rviz': rviz, 'rviz_software_gl': rviz_software_gl,
        'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y_pose})
    stack = TimerAction(period=stack_delay, actions=[
        include('localization.launch.py', {
            'map': map_yaml, 'use_sim_time': use_sim_time, 'params_file': nav2_params,
            'set_initial_pose': 'true', 'initial_x': x_pose, 'initial_y': y_pose,
            'initial_yaw': yaw}),
        include('perception.launch.py', {
            'params_file': perception_params, 'use_sim_time': use_sim_time}),
        include('navigation.launch.py', {
            'use_sim_time': use_sim_time, 'params_file': nav2_params, 'safety_gate': 'true'}),
    ])

    return LaunchDescription(declare_args + [sim, stack])
