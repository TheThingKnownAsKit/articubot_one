import os
from pathlib import Path

from ament_index_python import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name='articubot_one'
    robot_xml = xacro.process_file(
    os.path.join(get_package_share_directory('articubot_one'),
                 'description', 'robot.urdf.xacro')).toxml()

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # Launch Gazebo Sim (Harmonic)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            'gz_args': f'-v 4 -s libgz_ros_init.so -s libgz_ros_factory.so -s gz-sim-user-commands-system {os.path.join(get_package_share_directory(package_name), 'worlds', 'empty_plane.world')}'
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'articubot_one',
            '-topic', "robot_description",
            '-z' '0.1'
        ],
        parameters=[{
            "robot_description": robot_xml
        }]
    )

    # Bridge ROS2 Topics to Gazebo
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ]
    )
    lidar2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar2_bridge',
        output='screen',
        arguments=[
            '/lidar2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ]
    )

    return launch.LaunchDescription([
        robot_description_launch,
        gazebo_launch,
        spawn_entity,
        cmd_vel_bridge,
        lidar2_bridge
    ])
