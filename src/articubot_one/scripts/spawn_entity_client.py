#!/usr/bin/env python3
import os, rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class Spawner(Node):
    def __init__(self):
        super().__init__('spawn_entity_client')
        self.declare_parameter('world', 'empty_plane')
        self.declare_parameter('model_path', '')
        world       = self.get_parameter('world').value
        model_path  = self.get_parameter('model_path').value

        # Read SDF or URDF file
        with open(model_path, 'r') as f:
            sdf_xml = f.read()

        self.cli = self.create_client(
            SpawnEntity,
            f'/world/{world}/create')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /create…')

        req = SpawnEntity.Request()
        req.entity_factory.sdf = sdf_xml
        req.entity_factory.name = 'rover'
        req.entity_factory.pose.position.z = 0.10
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(
            lambda f: self.get_logger().info(
                'Spawn success' if f.result().success else 'Spawn failed')
        )

def main():
    rclpy.init()
    Spawner()
    rclpy.spin(rclpy.get_default_context().get_global_default_node())
if __name__ == '__main__':
    main()
