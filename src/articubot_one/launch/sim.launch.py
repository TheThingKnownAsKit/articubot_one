import os
from pathlib import Path

from ament_index_python import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Determine path to the robot URDF file
    pkg_prefix = get_package_share_directory("articubot_one")
    ros_gz_prefix = get_package_share_directory("ros_gz_sim")

    urdf_file  = os.path.join(pkg_prefix, "description", "robot.urdf.xacro")
    world_file = os.path.join(pkg_prefix, "worlds", "empty_plane.world")

    # Launch Gazebo Sim (Harmonic)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_prefix, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            'gz_args': f'-v 4 {world_file}'  # Launch empty world (-r for headless if you want no GUI; remove -r for GUI)
        }.items()
    )

    # Spawn the robot into Gazebo using ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'articubot_one', '-world', 'default', '-file', urdf_file],
        output='screen'
    )

    # Bridge /clock topic from Gazebo to ROS 2 (so ROS nodes use sim time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    # (Optional) Additional bridges can be added here for sensors or commands
    # e.g., bridging TF or joint states if needed:
    # tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/model/articubot_one/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
    #     output='screen'
    # )

    return launch.LaunchDescription([
        gazebo_launch,
        spawn_entity,
        clock_bridge,
        # Include additional bridges or nodes here
    ])
