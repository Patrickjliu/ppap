#!/usr/bin/env python


import cv2
from dt_apriltags import Detector
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from mavros_msgs.msg import Altitude
from cv_bridge import CvBridge



class TagFollowingNode(Node):
    def __init__(self):
        super().__init__("tag_following_node")

        self.at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

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

        self.get_logger().info("started desired heading subscription")
        

        self.dist_pub = self.create_publisher(
            Altitude, 
            "bluerov2/altitude", 
            10
        )

        self.get_logger().info("started distance away subscription")



    def tag_callback(self, msg):
        """
        This callback method takes an image and detects for april tags
        if there are april tags, the method will choose one of them and calculate its distance (m) from the rov
        using this information, we publish information to control the rov to move towards the tag
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

            # get the nearest tag
            closest_tag = tags[0]
            for tag in tags:
                if self.get_tag_distance(tag) < closest_tag:
                    closest_tag = tag

            # calculates, logs, and publishes distance to the closest tag
            distance = self.get_tag_distance(closest_tag)
            self.get_logger().info(f"Distance from April Tag: {distance}")
            self.publish_distance(distance)

            # calculates, logs, and publishes the angle difference from current and desired path
            angle = self.get_tag_angle(closest_tag)
            self.get_logger().info(f"Desired heading: {np.rad2deg(angle)}")
            self.publish_desired_heading(int(np.rad2deg(angle)))

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
        this method takes an image file and returns a list of tags that it detected
        """
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(img, estimate_tag_pose=True, camera_params=[2218, 625, self.width / 2, self.height / 2], tag_size=0.1)
        return tags

    def publish_desired_heading(self, desired_heading):
        '''
        takes the desired heading and publishes it to bluerov2/desired_heading
        '''
        aaa = Int16()
        aaa.data = desired_heading
        self.heading_pub.publish(aaa)



    def get_tag_distance(self, tag):
        """
        return the translational vector of how far away the tag is
        in an array
        """
        position = tag.pose_t
        return (np.sqrt(position[0]**2+position[1]**2+position[2]**2))
    
    def get_tag_angle(self, tag):
        """
        return the translational vector of how far away the tag is
        in an array
        """
        position = tag.pose_t
        return np.arctan(position[0]/position[1])




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
