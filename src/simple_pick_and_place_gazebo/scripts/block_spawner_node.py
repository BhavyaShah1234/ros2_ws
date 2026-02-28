#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import random
import math
import time


class BlockSpawnerNode(Node):
    def __init__(self):
        super().__init__('block_spawner_node')
        
        # Declare parameters
        self.declare_parameter('num_blocks', 10)
        self.declare_parameter('spawn_radius', 0.4)
        self.declare_parameter('spawn_height', 0.85)
        self.declare_parameter('colors', ['red', 'green', 'blue'])
        self.declare_parameter('shapes', ['cylinder', 'cuboid', 'sphere', 'cone'])
        
        # Get parameters
        self.num_blocks = self.get_parameter('num_blocks').value
        self.spawn_radius = self.get_parameter('spawn_radius').value
        self.spawn_height = self.get_parameter('spawn_height').value
        self.colors = self.get_parameter('colors').value
        self.shapes = self.get_parameter('shapes').value
        
        # Create service client for spawning entities
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/world/pick_and_place/create'
        )
        
        # Wait for service
        self.get_logger().info('Waiting for spawn service...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Spawn blocks
        self.spawn_blocks()
        
    def spawn_blocks(self):
        """Spawn random blocks in the workspace"""
        
        self.get_logger().info(f'Spawning {self.num_blocks} blocks...')
        
        for i in range(self.num_blocks):
            # Random color and shape
            color = random.choice(self.colors)
            shape = random.choice(self.shapes)
            model_name = f'{color}_{shape}'
            
            # Random position within spawn radius (circular distribution)
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(0.1, self.spawn_radius)
            x = 0.5 + radius * math.cos(angle)  # Offset to table center
            y = radius * math.sin(angle)
            z = self.spawn_height
            
            # Random yaw orientation
            yaw = random.uniform(0, 2 * math.pi)
            
            # Create unique entity name
            entity_name = f'block_{i}_{color}_{shape}'
            
            # Prepare SDF string
            sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <include>
    <uri>model://{model_name}</uri>
    <name>{entity_name}</name>
    <pose>{x} {y} {z} 0 0 {yaw}</pose>
  </include>
</sdf>'''
            
            # Create request
            request = SpawnEntity.Request()
            request.name = entity_name
            request.xml = sdf
            
            # Call service
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(
                        f'Spawned {entity_name} at ({x:.3f}, {y:.3f}, {z:.3f})'
                    )
                else:
                    self.get_logger().warn(
                        f'Failed to spawn {entity_name}: {future.result().status_message}'
                    )
            else:
                self.get_logger().error(f'Service call failed for {entity_name}')
            
            # Small delay between spawns
            time.sleep(0.1)
        
        self.get_logger().info('Block spawning complete!')


def main(args=None):
    rclpy.init(args=args)
    node = BlockSpawnerNode()
    # Node will exit after spawning blocks
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
