#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import argparse

class StartCoordinator(Node):
    def __init__(self, mission_ns='mission'): # 50 seconds offset
        super().__init__('start_coordinator')
        self.mission_ns = mission_ns
        self.declare_parameters(
            namespace='',# wildcard for all namespaces
            parameters=[
                ('start_offset_sec', rclpy.Parameter.Type.DOUBLE)
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

    def publish_once(self):
        now = self.get_clock().now()
        start_time = (now + rclpy.time.Duration(seconds=self.start_offset_sec)).to_msg()
        self.start_pub.publish(start_time)
        self.get_logger().info(
            f'Published start_time={start_time.sec}.{start_time.nanosec:09d} (offset={self.start_offset_sec}s)'
        )
        self.timer.cancel()

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