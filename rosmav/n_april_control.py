#!/usr/bin/env python

import cv2
from dt_apriltags import Detector
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from mavros_msgs.msg import ManualControl, Altitude, OverrideRCIn

kill_range = 2
opp_ids = [23, 16, 3, 4]

class TagFollowingNode(Node):
    def __init__(self):
        super().__init__("tag_following_node")

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

        self.get_logger().info("started camera subscription")

        self.heading_pub = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

        self.motion_pub = self.create_publisher(ManualControl, "bluerov2/manual_control", 10)

        self.get_logger().info("started desired heading publisher")
        
        self.lights = self.create_publisher(
            OverrideRCIn,
            "/bluerov2/override_rc",
            10
        )

        self.get_logger().info("started desired lights publisher")
        
        self.dist_pub = self.create_publisher(
            Altitude, 
            "bluerov2/altitude", 
            10
        )

        self.get_logger().info("started distance away subscription")


    def tag_callback(self, msg):
        """
        This callback method takes an image and detects for april tags.
        If there are april tags, the method will choose one of them and calculate its distance (m) from the ROV.
        Using this information, we publish information to control the ROV to move towards the tag.
        """

        if msg is None:
            self.get_logger().info("yes empty")
            return

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        self.width = msg.width
        self.height = msg.height
        tags = self.detect_tags(img)

        if len(tags) > 0:
            self.get_logger().info("tags detected.")

            # Draw and save tags on the image
            if len(img.shape) == 2:  # If image is grayscale
                color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                color_img = img

            self.draw_and_save_tags(color_img, tags)

            # Get the nearest tag
            closest_tag = tags[0]
            for tag in tags:
                if self.get_tag_distance(tag) < self.get_tag_distance(closest_tag):
                    closest_tag = tag

            # Calculates, logs, and publishes distance to the closest tag
            distance = self.get_tag_distance(closest_tag)
            # self.get_logger().info(f"Distance from April Tag: {distance}")
            self.publish_distance(distance)

            # Calculates, logs, and publishes the angle difference from current and desired path
            angle = self.get_tag_angle(closest_tag)
            self.get_logger().info(f"Desired heading: {np.rad2deg(angle)}")
            self.publish_desired_heading(int(np.rad2deg(angle)))
            
            if distance < kill_range and closest_tag.tag_id in opp_ids:
                self.flash()

        else:
            self.publish_desired_heading(90)
            self.get_logger().info("No tags detected.")
    
    def publish_distance(self, dist):
        '''
        Takes the distance that the robot has to travel, and publishes it to bluerov2/altitude.local
        '''    

        bbb = Altitude()
        bbb.local = float(dist)
        bbb.header.stamp = self.get_clock().now().to_msg()
        self.dist_pub.publish(bbb)
        self.get_logger().info(f"current difference: {dist}")
        
    def detect_tags(self, img):
        """
        This method takes an image file and returns a list of tags that it detected.
        """
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(img, estimate_tag_pose=True, camera_params=[1017, 1017, self.width / 2, self.height / 2], tag_size=0.1)
        return tags

    def publish_desired_heading(self, desired_heading):
        '''
        Takes the desired heading and publishes it to bluerov2/desired_heading.
        '''
        aaa = Int16()
        aaa.data = desired_heading
        self.heading_pub.publish(aaa)

    def get_tag_distance(self, tag):
        """
        Returns the translational vector of how far away the tag is.
        """
        position = tag.pose_t
        return (np.sqrt(position[0]**2 + position[1]**2 + position[2]**2))/2.8
    
    def get_tag_angle(self, tag):
        """
        Returns the angle difference from the tag.
        """
        position = tag.pose_t
        return np.arctan(position[0] / position[1])
    
    def draw_and_save_tags(self, img, tags):
        """
        Draws a box around each detected tag and saves the image.
        """
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(img, 
                         tuple(tag.corners[idx - 1, :].astype(int)), 
                         tuple(tag.corners[idx, :].astype(int)), 
                         (0, 255, 0), 2)

            cv2.putText(img, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255), thickness=2)

        filename = 'detected_tags.png'
        cv2.imwrite(filename, img)
        self.get_logger().info(f"Saved image with detected tags as {filename}")
    
    def flash(self):
        """Flashes lights at the AUV"""
        #Maintains middle depth
        RCmovement = OverrideRCIn()
        msg = Altitude()
        #  self.PID_desired_depth_publisher.publish(msg)

        #sets up a timer, lights_on is a counter, each 10 frames the light either turns on or turns off
        for i in range(1,20):
            if i % 2 == 0:
                RCmovement.channels[8] = 2000
                RCmovement.channels[9] = 2000   
                # self.get_logger().info("LIGHTS ON") 
                self.lights.publish(RCmovement)
            elif i % 2 == 1:
                RCmovement.channels[8] = 1000
                RCmovement.channels[9] = 1000
                self.lights.publish(RCmovement)
            
        #make sure that we stop when we're flashing the lights
        ManualMovement = ManualControl()
        ManualMovement.x = 0.0
        self.motion_pub.publish(ManualMovement)

def main(args=None):
    rclpy.init(args=args)

    node = TagFollowingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()