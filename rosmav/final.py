import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dt_apriltags import Detector
import cv2
import numpy as np

class CombinedNode(Node):
    def __init__(self):
        super().__init__("combined_node")

        # Square movement initialization
        self.motion_pub = self.create_publisher(ManualControl, "bluerov2/manual_control", 10)
        self.heading_pub = self.create_publisher(Int16, "bluerov2/desired_heading", 10)
        self.state = 'MOVE_FORWARD'
        self.start_time = self.get_clock().now()
        self.duration_move = Duration(seconds=3.0)  # Duration to move forward
        self.duration_turn = Duration(seconds=1.0)  # Duration to turn
        # self.square_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Square movement initialized.")

        # Tag detection initialization
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.tag_sub = self.create_subscription(
            Image, 
            "bluerov2/camera",
            self.tag_callback,
            10
        )

        self.lights = self.create_publisher(OverrideRCIn, "/bluerov2/override_rc", 10)
        self.dist_pub = self.create_publisher(Altitude, "bluerov2/altitude", 10)

        self.get_logger().info("Tag detection initialized.")

        self.kill_range = 1
        self.opp_ids = [6, 7, 3, 10]
        self.tag_detected = False
        self.bridge = CvBridge()

    def control_loop(self):
        if not self.tag_detected:
            current_time = self.get_clock().now()
            
            if self.state == 'MOVE_FORWARD':
                if (current_time - self.start_time) < self.duration_move:
                    self.publish_control_command(forward_power=70.0, yaw_power=0.0)
                    self.get_logger().info("Moving straight.")
                else:
                    self.state = 'TURN_RIGHT'
                    self.start_time = current_time
                    self.publish_heading_command(90)

            elif self.state == 'TURN_RIGHT':
                if (current_time - self.start_time) < self.duration_turn:
                    self.publish_control_command(forward_power=0.0, yaw_power=-100.0)
                    self.get_logger().info("Turning right.")
                else:
                    self.state = 'MOVE_FORWARD'
                    self.start_time = current_time
                
    def calculate_contrast(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(gray, cv2.CV_64F).var()


    def tag_callback(self, msg):
        if msg is None:
            self.get_logger().info("Received empty image message.")
            return

        img = self.bridge.imgmsg_to_cv2(msg)
        self.width = msg.width
        self.height = msg.height

        # Basic CV
        # contrast = self.calculate_contrast(img)
        # if contrast > 100:  # Adjust threshold as needed
        #     self.get_logger().info("High contrast detected, slowing down.")
        #     self.publish_control_command(forward_power=250.0, yaw_power=0.0)  # Slow down
        #     rclpy.spin_once(self, timeout_sec=2.0)  # Give time for robot to lock onto target

        tags = self.detect_tags(img)

        if len(tags) > 0:
            self.tag_detected = True
            self.get_logger().info("Tags detected.")

            closest_tag = tags[0]
            for tag in tags:
                if self.get_tag_distance(tag) < self.get_tag_distance(closest_tag):
                    closest_tag = tag

            distance = self.get_tag_distance(closest_tag)
            self.publish_distance(distance)
            angle = self.get_tag_angle(closest_tag)
            self.publish_desired_heading(int(np.rad2deg(angle)))
            self.get_logger().info("Distance to tag: {distance}")
            if distance < self.kill_range and closest_tag.tag_id in self.opp_ids:
                self.flash()
                self.stop_movement()
        else:
            self.tag_detected = False
            self.get_logger().info("No tags detected.")
            self.control_loop()
            

    def detect_tags(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(img, estimate_tag_pose=True, camera_params=[1017, 1017, self.width / 2, self.height / 2], tag_size=0.1)
        return tags

    def publish_distance(self, dist):
        msg = Altitude()
        msg.local = float(dist)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.dist_pub.publish(msg)
        self.get_logger().info(f"Current distance: {dist}")

    def publish_desired_heading(self, desired_heading):
        msg = Int16()
        msg.data = desired_heading
        self.heading_pub.publish(msg)

    def get_tag_distance(self, tag):
        position = tag.pose_t
        return np.sqrt(position[0]**2 + position[1]**2 + position[2]**2) / 2.8

    def get_tag_angle(self, tag):
        position = tag.pose_t
        return np.arctan(position[0] / position[1])

    def flash(self):
        RCmovement = OverrideRCIn()
        for i in range(1, 20):
            if i % 2 == 0:
                RCmovement.channels[8] = 2000
                RCmovement.channels[9] = 2000   
                self.lights.publish(RCmovement)
            else:
                RCmovement.channels[8] = 1000
                RCmovement.channels[9] = 1000
                self.lights.publish(RCmovement)
        

    def publish_control_command(self, forward_power, yaw_power):
        command = ManualControl()
        command.header.stamp = self.get_clock().now().to_msg()
        command.x = forward_power
        command.r = yaw_power
        self.motion_pub.publish(command)

    def publish_heading_command(self, heading):
        msg = Int16()
        msg.data = heading
        self.heading_pub.publish(msg)

    def stop_movement(self):
        command = ManualControl()
        command.header.stamp = self.get_clock().now().to_msg()
        command.x = 0.0
        command.y = 0.0
        command.z = 0.0
        command.r = 0.0
        self.motion_pub.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()

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