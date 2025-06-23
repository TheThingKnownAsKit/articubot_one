import os
from pathlib import Path

import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Determine path to the robot URDF file
    pkg_share = os.environ.get('COLCON_PREFIX_PATH', '').split(':')
    # Find the first path that ends with our package name
    pkg_path = None
    for p in pkg_share:
        if p.endswith('articubot_one'):
            pkg_path = p
            break
    urdf_file = os.path.join(pkg_path, 'share/articubot_one/description/robot.urdf.xacro')
    assert os.path.exists(urdf_file), f"URDF file not found: {urdf_file}"

    # Launch Gazebo Sim (Harmonic)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([Path(os.environ['COLCON_PREFIX_PATH'].split(':')[0]) / 'share' / 'ros_gz_sim' / 'launch' / 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'.format()  # Launch empty world (-r for headless if you want no GUI; remove -r for GUI)
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
