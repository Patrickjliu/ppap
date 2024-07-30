#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.cvb = CvBridge()
        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )

    def image_callback(self, msg: Image):
        # Convert Image message to OpenCV image
        image = self.cvb.imgmsg_to_cv2(msg)

    def process_image(self, img):
        # Lane detection
        lines = detect_lines(img)
        if lines is not None:
            slopes, _ = get_slopes_intercepts(lines)
            return heading
        return "forward"

def main(args=None):
    rclpy.init(args=args)
    lane_follower = LaneFollower()
    try:
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        pass
    finally:
        lane_follower.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

def detect_lines(img, threshold1=1, threshold2=10, apertureSize=3, minLineLength=100, maxLineGap=10):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertureSize)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=minLineLength, maxLineGap=maxLineGap)
    return lines

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 != 0:
            slope = (y2 - y1) / (x2 - x1)
            slopes.append(slope)
            intercepts.append(y1 - slope * x1)
    return slopes, intercepts

def follow_lane(slopes):
    if not slopes:
        return "forward"
    steepest_slope = max(slopes, key=abs)
    if steepest_slope > 0:
        return "left"
    elif steepest_slope < 0:
        return "right"
    else:
        return "forward"