#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl
from std_srvs.srv import SetBool
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, integral_limit=100.0):
        """
        Initialize PID controller.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, measurement, dt):
        """
        Compute control signal.
        """
        if dt <= 0:
            return 0.0

        error = self.setpoint - measurement
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.last_error) / dt
        self.last_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative

class DepthController(Node):
    """
    Control depth using PID.
    """

    def __init__(self):
        """
        Initialize DepthController node.
        """
        super().__init__('depth_controller')

        self.pid = PID(kp=10.0, ki=3, kd=1.3, setpoint=0.6)

        self.manual_control_pub = self.create_publisher(
            ManualControl, 'bluerov2/manual_control', 10
        )
        self.desired_depth_sub = self.create_subscription(
            Float64, 'bluerov2/desired_depth', self.desired_depth_callback, 10
        )
        self.altitude_sub = self.create_subscription(
            Altitude, 'bluerov2/attitude', self.altitude_callback, 10
        )

        self.arming_client = self.create_client(SetBool, 'bluerov2/arming')

        self.last_time = None
        self.current_depth = 0.0
        self.desired_depth = 0.0
        # self.get_logger().error(f"1")

    def arm_vehicle(self, arm: bool):
        """
        Arm or disarm the vehicle.
        """
        # self.get_logger().error(f"2")
        request = SetBool.Request()
        request.data = arm
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info(f"Vehicle {'armed' if arm else 'disarmed'}")
        else:
            self.get_logger().error(f"Failed to {'arm' if arm else 'disarm'}")

    def set_vertical_power(self, power):
        """
        Set vertical thruster power using ManualControl message.
        """
        power = float(np.clip(power, -1000, 1000))
        manual_control_msg = ManualControl()
        manual_control_msg.z = -power
        self.manual_control_pub.publish(manual_control_msg)
        # self.get_logger().info(f" manual_control_msg: {manual_control_msg} ")

    def desired_depth_callback(self, msg: Float64):
        """
        Update desired depth.
        """
        self.desired_depth = msg.data
        self.pid.setpoint = self.desired_depth
        # self.get_logger().info(f" self.desired_depth: {self.desired_depth} ")

    def altitude_callback(self, msg: Altitude):
        """
        Handle altitude updates and compute PID control.
        """
        
        self.current_depth = msg.relative

        # PID Controller

        # current_time = self.get_clock().now()
        # if self.last_time is None:
        #     self.last_time = current_time
        #     return

        # dt = (current_time - self.last_time).to_msg().nanosec / 1e9

        # if dt <= 0:
        #     self.get_logger().warn("Invalid time step.")
        #     return
        # control_signal = self.pid.update(self.current_depth, dt)
        # self.set_vertical_power(control_signal)

        # self.last_time = current_time

        # Bang-Bang Controller
        commands = ManualControl()
        if (self.current_depth - self.desired_depth) < 0.1:
            commands.z = -20.0
        elif (self.current_depth - self.desired_depth) > 0.1:
            commands.z = 20.0
        else:
            commands.z = 0.0
        self.manual_control_pub.publish(commands)
        self.get_logger().info(f"sent commands: {commands.z}")

def main(args=None):
    """
    Main entry point.
    """
    rclpy.init(args=args)
    depth_controller = DepthController()

    try:
        rclpy.spin(depth_controller)
    except KeyboardInterrupt:
        depth_controller.get_logger().info('Shutting down...')
    finally:
        depth_controller.arm_vehicle(False)
        depth_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
