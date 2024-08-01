#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude, OverrideRCIn, ManualControl
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from time import sleep


class OffensiveDepthControlNode(Node):

    def __init__(self):
        super().__init__("offensive_depth_control_node")
        self.vertical_power = 0
        self.times = [0, 0]
        self.error_accumulator = 0
        self.previous_error = 0.5
        self.safety = 0.5

        # self._step_counter = 0
	
        self.depth_sub = self.create_subscription(
            Altitude, 
            "bluerov2/altitude", 
            self.forward_callback,
            10
        )

        self.get_logger().info("starting depth subscriber")

        self.motion_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

        self.get_logger().info("starting control publisher")

        self.lights_pub = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )

        self.set_lights(False)
        self.get_logger().info("starting lights publisher")

        # self.loop = self.create_timer(1.0, self._loop)

    def depth_callback(self, msg):
        '''
        callback for depth usage, not being called anymore
        '''

        self.times[0] = self.times[1]
        self.times[1] = msg.header.stamp.sec
        self.get_logger().info(f"times: {self.times[0]}, {self.times[1]}")
        self.depth = msg.relative
        self.pid_calculations(-0.5)
        self.get_logger().info(f"current depth {self.depth}")
        
    
    def move_depth(self):
        '''
        callback to depreciated depth control feature
        ''' 

        commands = ManualControl()
        if abs(self.vertical_power) > 1:
            self.vertical_power = self.vertical_power / abs(self.vertical_power)
        commands.z = 50 + 30 * self.vertical_power
        self.motion_pub.publish(commands)
        self.get_logger().info(f"sent commands: {commands.z}")

    def forward_callback(self, msg):
        '''
        given a distance that the robot wishes to move in the message, makes the robot move based on the pid controller
        '''

        # time tracking for dt
        self.times[0] = self.times[1]
        self.times[1] = msg.header.stamp.sec + msg.header.stamp.nanosec / (10 ** 9)
        self.get_logger().info(f"times: {self.times[0]}, {self.times[1]}")

        # sets error to the message recieved by subsscriber
        self.error_diff = msg.local

        # if the target is within specified range, it shoots, otherwise, it moves closer using pid
        if abs(self.error_diff) > self.safety:
            self.get_logger().info("moving towards target")
            self.pid_calculations()
            self.get_logger().info(f"current difference {self.error_diff}")
            self.move_forward()

        else:
            self.get_logger().info("releasing torpedos!!!")
            for i in range(2):
                self.set_lights(True)
                sleep(0.5)
                self.set_lights(False)
                sleep(0.5)


    

    def set_lights(self, state: bool):
        '''
        sets lights on if given true, turns lights off if given false
        '''
        msg = OverrideRCIn()

        # checks the command passed through
        NC = 65535
        num_state = 1000
        if state:
            num_state = 1500
        
        # sets channels to no change except for lights
        msg.channels = [NC, NC, NC, NC, NC, NC, NC, num_state, num_state, NC, NC, NC, NC, NC, NC, NC, NC, NC]

        self.lights_pub.publish(msg)


    def move_forward(self):
        '''
        uses manual control to move forward
        '''

        commands = ManualControl()

        # hard limit for pid
        if abs(self.forward_power) > 1:
            self.forward_power = self.forward_power / abs(self.forward_power)

        # scaling for power
        commands.x = 50 + 30 * self.forward_power

        self.motion_pub.publish(commands)
        self.get_logger().info(f"sent commands: {commands.x}")

    def pid_calculations(self):
        '''
        pid controller
        '''

        # proportion
        Kp = 1.5
        error = self.error_diff
        proportional = Kp * error
        self.get_logger().info(f"error: {error}")
        self.get_logger().info(f"p: {proportional}")

        # integral (currently ignored)
        Ki = 0
        dt = self.times[1] - self.times[0]
        self.error_accumulator += error * dt # dt is the time since the last update
        integral = min(Ki * self.error_accumulator, 1.0)
        self.get_logger().info(f"i: {integral}")

        # derivative
        Kd = 2.0
        derivative = Kd * (error - self.previous_error) / dt # dt is the time since the last update
        self.previous_error = error
        self.get_logger().info(f"d: {derivative}")

        self.forward_power = proportional + derivative
        
	
	
	

   
def main(args=None):
    rclpy.init(args=args)
    node = OffensiveDepthControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # Cleanup
        node.set_lights(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()