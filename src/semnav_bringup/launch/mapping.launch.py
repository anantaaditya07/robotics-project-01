"""Mode B (mapping demo): simulation + slam_toolbox online_async.

Starts the full simulation (sim.launch.py: Gazebo Classic, SemNav waffle, optional RViz) and
slam_toolbox in online_async mode to build a map from /scan (frame base_scan, 3.5 m max range)
and the odom -> base_footprint TF (D-07). slam_toolbox publishes /map and map -> odom.

Usage:
  ros2 launch semnav_bringup mapping.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive around to build the map
  scripts/save_map.sh [name]                              # save to src/semnav_bringup/maps/
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_software_gl = LaunchConfiguration('rviz_software_gl')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_args = [
        DeclareLaunchArgument(
            'gui', default_value='true', description='Start the Gazebo client (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_software_gl', default_value='false',
            description='Render RViz with Mesa software GL (D-17: Map shader fails on Intel iGPU)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'slam_toolbox_online_async.yaml'),
            description='slam_toolbox parameter file'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': gui,
            'rviz': rviz,
            'rviz_software_gl': rviz_software_gl,
            'use_sim_time': use_sim_time,
        }.items())

    # Same as slam_toolbox 2.6.10 online_async_launch.py: plain (non-lifecycle) node.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription(declare_args + [
        sim,
        slam_toolbox,
    ])
