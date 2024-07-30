#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class HeadingSetterNode(Node):
    def __init__(self):
        super().__init__("heading_setter")

        self.declare_parameter("desired_heading", 0)
        desired_heading = self.get_parameter("desired_heading").value

        self.publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.publish_desired_heading(desired_heading)

        self.get_logger().info("starting publisher node")

    def publish_desired_heading(self, desired_heading):
        msg = Int16()
        msg.data = desired_heading


        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingSetterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()