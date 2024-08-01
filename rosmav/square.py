import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
import time

class SquareMovementNode(Node):

    def __init__(self):
        super().__init__("square_movement_node")
        self.motion_pub = self.create_publisher(ManualControl, "bluerov2/manual_control", 10)

        self.heading_pub = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.loop = self.create_timer(5.0, self.move_in_square)

        self.get_logger().info("Square movement node has started.")

    def move_in_square(self):
        try:
            while rclpy.ok():
                # Move straight
                self.publish_control_command(forward_power=500, yaw_power=0)
                self.get_logger().info("Moving straight.")
                time.sleep(3)  # Move straight for 3 seconds

                msg = Int16()
                msg.data = int(desired_angle)
                self.heading_pub.publish(msg)

                # Stop
                self.stop_movement()
                self.get_logger().info("Stopping.")
                time.sleep(0.5)  # Pause briefly between movements

        except KeyboardInterrupt:
            self.get_logger().info("KeyboardInterrupt received, stopping the movement.")
            self.stop_movement()

    def publish_control_command(self, forward_power, yaw_power):
        command = ManualControl()
        command.x = forward_power  # Forward/backward
        command.r = yaw_power      # Yaw (rotation)
        self.motion_pub.publish(command)

    def stop_movement(self):
        self.publish_control_command(forward_power=0, yaw_power=0)


def main(args=None):
    rclpy.init(args=args)
    node = SquareMovementNode()

    try:
        node.move_in_square()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
