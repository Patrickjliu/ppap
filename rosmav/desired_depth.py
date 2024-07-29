#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Float64


class DesiredDepth(Node):
    def __init__(self):
        super().__init__("DesiredDepth")
        
        # Create a publisher for the temperature message
        self.desired_depth_pub = self.create_publisher(
            Float64, "bluerov2/desired_depth", 10
        )


def main(args=None):
    rclpy.init(args=args)
    node = DesiredDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
