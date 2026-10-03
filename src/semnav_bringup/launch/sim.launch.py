"""Phase 1 simulation bringup: Gazebo Classic + TurtleBot3 Waffle + optional RViz.

Uses the stock turtlebot3_gazebo world and robot model. Phase 2 replaces the robot model
with a SemNav copy (camera 640x480 @ 15 Hz, optical frame) and the world with a SemNav world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Start the Gazebo client (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'semnav.rviz'),
            description='RViz2 config file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'robot_model', default_value='waffle',
            description='TURTLEBOT3_MODEL used by the turtlebot3_gazebo launch files'),
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0', description='Robot spawn x in the world frame (m)'),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.5', description='Robot spawn y in the world frame (m)'),
    ]

    # The turtlebot3_gazebo launch files read TURTLEBOT3_MODEL from the environment.
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz))

    return LaunchDescription(declare_args + [
        set_model,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
