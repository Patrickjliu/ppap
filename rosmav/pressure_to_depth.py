#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import Altitude
from std_msgs.msg import Header

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')

        self.altitude_pub = self.create_publisher(Altitude, "bluerov2/attitude", 10)
        self.pressure_sub = self.create_subscription(
            FluidPressure, 'bluerov2/pressure', self.pressure_callback, 10
        )

        self.last_time = self.get_clock().now()
        self.static_pressure = 0.0

    def pressure_callback(self, msg: FluidPressure):
        self.static_pressure = msg
        self.publish_depth()

    def publish_depth(self):
        if self.static_pressure is None:
            self.get_logger().warn("Pressure data missing.")
            return

        atm = 104250.0  # Atmospheric pressure in Pa
        water_density = 1000  # kg/m^3
        gravity = 9.81  # m/s^2
        static_pressure = self.static_pressure.fluid_pressure

        depth = (static_pressure - atm) / (water_density * gravity)
        self.get_logger().info(f"Publishing Depth: {depth:.2f} meters")

        current_time = self.get_clock().now()
        altitude_msg = Altitude()
        altitude_msg.header = Header()
        altitude_msg.header.stamp = current_time.to_msg()
        altitude_msg.relative = depth

        self.altitude_pub.publish(altitude_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_publisher = DepthPublisher()

    try:
        rclpy.spin(depth_publisher)
    except KeyboardInterrupt:
        depth_publisher.get_logger().info('Shutting down...')
    finally:
        depth_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
