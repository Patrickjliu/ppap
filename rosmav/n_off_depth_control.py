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

        self.declare_parameter("desired_depth", 0.5)
        self.desired_depth = self.get_parameter("desired_depth").value
	
        self.depth_sub = self.create_subscription(
            Altitude, 
            "bluerov2/altitude", 
            self.depth_callback,
            10
        )

        self.get_logger().info("starting depth subscriber")

        self.motion_pub = self.create_publisher(
            ManualControl, "bluerov2/manual_control", 10
        )

        self.get_logger().info("starting control publisher")
        # self.loop = self.create_timer(1.0, self._loop)

    def depth_callback(self, msg):
        '''
        callback for depth usage
        '''

        '''self.times[0] = self.times[1]
        self.times[1] = msg.header.stamp.sec
        self.get_logger().info(f"times: {self.times[0]}, {self.times[1]}")
        self.pid_calculations(-0.5)'''

        self.depth = msg.relative
        self.get_logger().info(f"current depth {self.depth}")
        
    
    def move_depth(self):
        '''
        callback to depreciated depth control feature
        ''' 

        commands = ManualControl()
        '''if abs(self.vertical_power) > 1:
            self.vertical_power = self.vertical_power / abs(self.vertical_power)
        commands.z = 50 + 30 * self.vertical_power'''

        if (self.depth - self.desired_depth) < 0.1:
            commands.z = -20
        elif (self.depth - self.desired_depth) > 0.1:
            commands.z = 20
        else:
            commands.z = 0
        self.motion_pub.publish(commands)
        self.get_logger().info(f"sent commands: {commands.z}")
    

  
   
def main(args=None):
    rclpy.init(args=args)
    node = OffensiveDepthControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        # Cleanup
        # node.set_lights(False)
        # node.turn_off_rc
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()