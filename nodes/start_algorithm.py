#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import argparse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class StartCoordinator(Node):
    def __init__(self, mission_ns='mission'): # 50 seconds offset
        super().__init__('start_coordinator')
        self.mission_ns = mission_ns
        self.declare_parameters(
            namespace='',# wildcard for all namespaces
            parameters=[
                ('start_offset_sec', rclpy.Parameter.Type.DOUBLE),
                ('obstacle_limits_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('obstacle_limits_y', rclpy.Parameter.Type.DOUBLE_ARRAY)
            ]
        )
        self.start_offset_sec = float(self.get_parameter('start_offset_sec').value)

        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.start_pub = self.create_publisher(Time, f'/{mission_ns}/start', start_qos)

        # Veröffentliche Startzeit einmal, kurz in der Zukunft
        self.timer = self.create_timer(0.2, self.publish_once)

        # Initialize obstacle boundaries with proper QoS
        obstacle_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.obstacle_boundaries_marker: Marker
        self.init_obstacle_boundaries_marker()
        self.obstacle_limits_x = self.get_parameter('obstacle_limits_x').value
        self.obstacle_limits_y = self.get_parameter('obstacle_limits_y').value
        self.obstacle_boundaries_pub = self.create_publisher(Marker, '~/obstacle_boundaries', obstacle_qos)
        
        # Timer for repeated obstacle boundaries publishing
        self.obstacle_timer = self.create_timer(1.0, self.publish_obstacle_boundaries)
        self.obstacle_publish_count = 0

    def publish_once(self):
        now = self.get_clock().now()
        start_time = (now + rclpy.time.Duration(seconds=self.start_offset_sec)).to_msg()
        self.start_pub.publish(start_time)
        self.get_logger().info(
            f'Published start_time={start_time.sec}.{start_time.nanosec:09d} (offset={self.start_offset_sec}s)'
        )
        self.timer.cancel()

    def init_obstacle_boundaries_marker(self):
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = 'obstacle_boundaries'
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.header.frame_id = 'map'
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.scale.x = 0.03
        msg.scale.y = 0.03
        msg.scale.z = 0.03
        self.obstacle_boundaries_marker = msg

    def publish_obstacle_boundaries(self):
        """Publish obstacle boundaries as line strip markers"""
        # Only publish first 5 times to avoid spam, but ensure delivery
        if self.obstacle_publish_count >= 5:
            self.obstacle_timer.cancel()
            return
            
        msg = self.obstacle_boundaries_marker
        
        # Create boundary points from obstacle limits
        boundary_points = []
        
        # Add rectangle boundary points (clockwise)
        x_min, x_max = self.obstacle_limits_x[0], self.obstacle_limits_x[1]
        y_min, y_max = self.obstacle_limits_y[0], self.obstacle_limits_y[1]
        
        # Bottom edge (left to right)
        boundary_points.append(Point(x=x_min, y=y_min, z=-0.5))
        boundary_points.append(Point(x=x_max, y=y_min, z=-0.5))
        
        # Right edge (bottom to top)
        boundary_points.append(Point(x=x_max, y=y_max, z=-0.5))
        
        # Top edge (right to left)
        boundary_points.append(Point(x=x_min, y=y_max, z=-0.5))
        
        # Left edge (top to bottom) - close the loop
        boundary_points.append(Point(x=x_min, y=y_min, z=-0.5))
        
        msg.points = boundary_points
        msg.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_boundaries_pub.publish(msg)
        
        self.obstacle_publish_count += 1
        self.get_logger().info(f'Published obstacle boundaries for visualization (attempt {self.obstacle_publish_count})')

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission_ns', default='mission')
    args, _ = parser.parse_known_args()

    node = StartCoordinator(mission_ns=args.mission_ns)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()