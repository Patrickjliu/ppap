#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure as Pressure
from mavros_msgs.msg import Altitude


class DepthNode(Node):
    def init(self):
        super().init("dancing_node")

        # self._step_counter = 0

    self.pressure_sub = self.create_subscriber(
        Pressure, 
        "bluerov2/pressure", 
        self.pressure_callback,
        10
    )

    self.get_logger().info("starting pressure subscriber")

        self.depth_pub = self.create_publisher(
            OverrideRCIn, 
        "bluerov2/override_rc", 
        10
        )

    self.pub_timer = self.create_timer(
        1.0, self.depth_callback
    )

    self.get_logger().info("starting depth publisher")

        # self.loop = self.create_timer(1.0, self.loop)

   def pressurecallback(self, msg):
    pwessuw = msg.fluidpressure
    self.depth = pwessuw / (1000 * 9.81)
    self.getlogger().info(f"current depth: {self.depth}")

   def depth_callback(self):
    msg = Altitude()
    msg.relative = self.depth
    self.publish,publish(msg)
    self.get_logger().info("depth message sent")





def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # Cleanup
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name == '__main':
    main()