#!/usr/bin/env python3
"""
sim_robot.launch.py  –  Gazebo Harmonic + ROS 2

* Launch Gazebo (server + GUI) with gz_sim.launch.py
* Start one ros_gz_bridge instance configured from YAML
* Spawn a model with gz_spawn_model.launch.py
"""

import os, xacro, tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ────────────────────────────── package paths
    pkg_bot   = get_package_share_directory('articubot_one')
    pkg_rosgz = get_package_share_directory('ros_gz_sim')
    pkg_bridge = get_package_share_directory('ros_gz_bridge')

    # ── expand Xacro → URDF *once* and write to a temp file
    xacro_file = os.path.join(pkg_bot, 'description', 'robot.urdf.xacro')
    urdf_xml   = xacro.process_file(xacro_file).toxml()
    tmp_urdf   = tempfile.NamedTemporaryFile(delete=False,
                                             suffix='.urdf',
                                             prefix='articubot_')
    tmp_urdf.write(urdf_xml.encode())
    tmp_urdf.close()                       # keep the file on disk
    urdf_path = tmp_urdf.name              # path we’ll hand to the spawner

    # ────────────────────────────── command-line arguments
    declare_world = DeclareLaunchArgument(
        'world', default_value='empty_plane.world',
        description='World SDF file, relative to my_robot_pkg/worlds')

    # ────────────────────────────── Gazebo (server + GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rosgz, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            # docs: pass extra cmd-line args with `gz_args` :contentReference[oaicite:0]{index=0}
            'gz_args': PathJoinSubstitution([
                '-r', pkg_bot, 'worlds', LaunchConfiguration('world')
            ])
        }.items())

    # ────────────────────────────── ROS–Gazebo bridge from YAML
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bridge, 'launch',
                                  'ros_gz_bridge.launch.py'])),
        launch_arguments={
            # docs: bridge_name / config_file arguments :contentReference[oaicite:1]{index=1}
            'bridge_name': 'ros_gz_bridge',
            'config_file': PathJoinSubstitution([
                pkg_bot, 'config', 'bridge_ros2_control.yaml'
            ])
        }.items())

    # ────────────────────────────── Spawn the robot
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rosgz,
                                   'launch', 'gz_spawn_model.launch.py'])),
        launch_arguments={
            # docs: required arguments world / file / entity_name :contentReference[oaicite:2]{index=2}
            'world':      'empty_plane',
            'file': urdf_path,
            'entity_name': 'rover',
            'x': '0.0', 'y': '0.0', 'z': '0.10'
        }.items())
    
    # ────────────────────────────── Create a robot_state_publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_bot, 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ────────────────────────────── Spawn the controllers
    joint_broad_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_broad', '--controller-manager', '-c', '/controller_manager'
    ])
    diff_cont_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '-c', '/controller_manager'
    ])


    # ───────────────────────────────── Launch description
    return LaunchDescription([
        declare_world,
        gz_sim,
        bridge,
        spawn,
        rsp,
        joint_broad_node,
        diff_cont_node
    ])