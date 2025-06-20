import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name='articubot_one'
    
    # Create a robot_state_publisher node
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Create the gazebo_sim node
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'empty.world'),
            'gui': 'true'
        }.items()
    )

    # Spawn the robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-z', '0.1'
        ]
    )

    # Bridge desired ros2 input into Gazebo
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

    # Launch!
    return LaunchDescription([
        rsp,
        gazebo_sim,
        spawn_robot,
        cmd_vel_bridge,
        lidar2_bridge
    ])
