import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16

class SquareMovementNode(Node):

    def __init__(self):
        super().__init__("square_movement_node")
        self.motion_pub = self.create_publisher(ManualControl, "bluerov2/manual_control", 10)
        self.heading_pub = self.create_publisher(Int16, "bluerov2/desired_heading", 10)
        
        self.state = 'MOVE_FORWARD'
        self.start_time = self.get_clock().now()
        self.duration_move = Duration(seconds=3.0)  # Duration to move forward
        self.duration_turn = Duration(seconds=1.0)  # Duration to turn
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Square movement node has started.")

    def control_loop(self):
        current_time = self.get_clock().now()
        
        if self.state == 'MOVE_FORWARD':
            if (current_time - self.start_time) < self.duration_move:
                self.publish_control_command(forward_power=500.0, yaw_power=0.0)
                self.get_logger().info("Moving straight.")
            else:
                self.state = 'TURN_RIGHT'
                self.start_time = current_time
                self.publish_heading_command(90)

        elif self.state == 'TURN_RIGHT':
            if (current_time - self.start_time) < self.duration_turn:
                self.publish_control_command(forward_power=0.0, yaw_power=500.0)
                self.get_logger().info("Turning right.")
            else:
                self.state = 'MOVE_FORWARD'
                self.start_time = current_time

    def publish_control_command(self, forward_power, yaw_power):
        command = ManualControl()
        command.header.stamp = self.get_clock().now().to_msg()
        command.x = forward_power  # Forward/backward
        command.r = yaw_power      # Yaw (rotation)
        self.motion_pub.publish(command)

    def publish_heading_command(self, heading):
        msg = Int16()
        msg.data = heading
        self.heading_pub.publish(msg)

    def stop_movement(self):
        command = ManualControl()
        command.header.stamp = self.get_clock().now().to_msg()
        command.x = 0.0  # Forward/backward
        command.r = 0.0  # Yaw (rotation)
        self.motion_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = SquareMovementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.stop_movement()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
