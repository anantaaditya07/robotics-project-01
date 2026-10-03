"""Simulation bringup: Gazebo Classic + SemNav TurtleBot3 Waffle + optional RViz.

Spawns the SemNav copy of the waffle (models/semnav_waffle: camera 640x480 @ 15 Hz, image
frame_id camera_rgb_optical_frame; D-05, D-06) into the SemNav world (worlds/semnav_world.world:
TB3 world + /gazebo/model_states + person/chair/bottle; D-04, D-14). The object models are not
in git: fetch them once with scripts/fetch_models.sh, then rebuild.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('semnav_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_software_gl = LaunchConfiguration('rviz_software_gl')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    robot_sdf = LaunchConfiguration('robot_sdf')
    robot_entity = LaunchConfiguration('robot_entity')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_dir, 'worlds', 'semnav_world.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Start the Gazebo client (gzclient)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_software_gl', default_value='false',
            description='Render RViz with Mesa software GL (D-17: Map shader fails on Intel iGPU)'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_dir, 'rviz', 'semnav.rviz'),
            description='RViz2 config file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'robot_model', default_value='waffle',
            description='TURTLEBOT3_MODEL used by the turtlebot3 robot_state_publisher URDF'),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(bringup_dir, 'models', 'semnav_waffle', 'model.sdf'),
            description='Robot SDF spawned into Gazebo'),
        DeclareLaunchArgument(
            'robot_entity', default_value='waffle', description='Gazebo entity name of the robot'),
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0', description='Robot spawn x in the world frame (m)'),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.5', description='Robot spawn y in the world frame (m)'),
    ]

    # D-19: Fast DDS profile with a large shared-memory segment so camera images are not lost.
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(bringup_dir, 'config', 'fastdds_profile.xml'))

    # The turtlebot3_gazebo launch files read TURTLEBOT3_MODEL from the environment.
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model)

    # SemNav models (semnav_waffle) and the fetched, git-ignored object models. TB3 meshes and
    # the turtlebot3_world model come from turtlebot3_gazebo's own gazebo_model_path export.
    model_paths = [
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(bringup_dir, 'models')),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH', os.path.join(bringup_dir, 'models_external')),
    ]

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

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_entity, '-file', robot_sdf,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        # D-17: Mesa treats LIBGL_ALWAYS_SOFTWARE=0 as unset, so 'false' keeps hardware GL.
        additional_env={'LIBGL_ALWAYS_SOFTWARE': PythonExpression(
            ["'1' if '", rviz_software_gl, "'.lower() == 'true' else '0'"])},
        output='screen',
        condition=IfCondition(rviz))

    return LaunchDescription(declare_args + model_paths + [
        fastdds_profile,
        set_model,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
