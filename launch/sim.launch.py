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
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Create the gazebo_sim node
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {os.path.join(get_package_share_directory(package_name), "worlds", "empty.world")}'}.items() # remove "-r" for paused startup
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

    # Launch!
    return LaunchDescription([
        rsp,
        gazebo_sim,
        spawn_robot,
    ])
