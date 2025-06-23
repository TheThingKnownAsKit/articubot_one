import os, xacro, rclpy
from pathlib import Path

from ament_index_python import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

PKG   = 'articubot_one'
WORLD = os.path.join(get_package_share_directory(PKG), 'worlds', 'empty_plane.world')
URDF  = xacro.process_file(
            os.path.join(get_package_share_directory(PKG),
                         'description', 'robot.urdf.xacro')).toxml()

def pub_description(context, *args):
    rclpy.init()
    node = rclpy.create_node('robot_description_pub')
    qos = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, 'robot_description', qos)
    pub.publish(String(data=URDF))
    node.get_logger().info('Latched robot_description published')
    rclpy.shutdown()

pub_once = OpaqueFunction(function=pub_description)

# Launch Gazebo Sim (Harmonic)
gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), "launch", "gz_sim.launch.py")
    ),
    launch_arguments={
        'gz_args': f'-v 4 -s libgz_ros_init.so -s libgz_ros_factory.so -s gz-sim-user-commands-system {WORLD}'
    }.items()
)

# Spawn the robot and attach it to the robot_description topic
spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=[
        '-name', 'articubot_one',
        '-topic', "robot_description",
        '-z' '0.1'
    ],
    # parameters=[{
    #     "robot_description": robot_xml
    # }]
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

def generate_launch_description():
    return launch.LaunchDescription([
        gazebo_launch,
        pub_once,
        spawn_robot,
        cmd_vel_bridge,
        lidar2_bridge
    ])
