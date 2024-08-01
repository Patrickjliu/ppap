#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu 
from mavros_msgs.msg import ManualControl
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16



class HeadingControlNode(Node):

    def __init__(self):
        super().__init__("heading_control_node")
        # self._step_counter = 0
        self.target_heading = 340
        self.derivative = 0

        self.target_heading_sub = self.create_subscription(
            Int16,
            "bluerov2/desired_heading",
            self.target_callback,
            10
        )

        self.derivative_sub = self.create_subscription(
            Imu,
            "bluerov2/imu",
            self.derivative_callback,
            10
        )
	
        self.heading_sub = self.create_subscription(
            Int16, 
            "bluerov2/heading", 
            self.heading_callback,
            10
        )

        self.get_logger().info("starting heading subscriber")

        self.motion_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

        

        # self.loop = self.create_timer(1.0, self.move)

        self.get_logger().info("starting control publisher")
        self.get_logger().info(f"target heading: {self.target_heading}")

    def target_callback(self, msg):
        '''
        sets target_heading to the desired heading + current heading because I am lazy
        '''
        self.target_heading = msg.data + self.heading
        self.get_logger().info(f"target heading: {self.target_heading}")

    def derivative_callback(self, msg):
        '''
        sets the derivative to the current angular velocity of the robot around the z axis
        '''
        self.derivative = msg.angular_velocity.z
        self.get_logger().info(f"current derivative: {self.derivative}")

    def move(self):
        power = self.calc_power(self.target_heading - self.heading) + self.derivative
        commands = ManualControl()
        
        power *= 2

        if abs(power) > 100:
            power = (power / abs(power)) * 100

        # scaling code
        commands.r = power
        if np.isnan(commands.r):
            return
        self.get_logger().info(f"power: {power}")
        self.motion_pub.publish(commands)

    def heading_callback(self, msg):
        self.heading = msg.data
        self.get_logger().info(f"current heading: {self.heading}")
        self.move()

    


    def calc_power(self, heading):
        if np.isnan(self.f(heading)):
            return
        return abs(self.g(heading)) * (self.f(heading) / abs(self.f(heading)))

    def f(self, x):
        return (200 / np.pi) * np.arctan(np.tan((x / 360) * np.pi))

    def g(self, x):
        return np.sin(x / 115) * 100

def main(args=None):
    rclpy.init(args=args)
    node = HeadingControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # Cleanup
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()