import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_articubot = get_package_share_directory('articubot_one')
    pkg_ros_gz     = get_package_share_directory('ros_gz_sim')

    world_path  = os.path.join(pkg_articubot, 'worlds', 'empty_plane.world')
    world_name  = 'empty_plane'                      # ⇦ must match <world name="…">

    # ───────────────────────────────── Gazebo server / client
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            'gz_args': f'-v 4 {world_path}'  # Launch empty world (-r for headless if you want no GUI; remove -r for GUI)
        }.items()
    )

    # ───────────────────────────────── Robot State Publisher (sim time)
    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_articubot, 'launch','robot_state_publisher.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ───────────────────────────────── Spawn robot into correct world
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-world', world_name,
            '-z', '0.1'
        ]
    )

    # ───────────────────────────────── ROS‒Gazebo topic bridges
    bridge_cmd_vel = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='cmd_vel_bridge', output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']
    )

    bridge_lidar2 = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='lidar2_bridge', output='screen',
        arguments=['/lidar2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
    )

    bridge_clock = Node(                              # drives /clock for sim-time nodes
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',  output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock']
    )

    # ───────────────────────────────── Launch description
    return LaunchDescription([
        gazebo_launch,
        rsp_launch,

        # small delay to ensure /world/<name>/create exists
        TimerAction(period=3.0, actions=[spawn_robot]),

        bridge_clock,
        bridge_cmd_vel,
        bridge_lidar2
    ])