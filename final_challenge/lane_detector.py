#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge

from final_challenge.detect_lane_lines import detect_lane_lines


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            "/lane_debug",
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detection_output = detect_lane_lines(frame)
        cam_img = detection_output["image"]

        out_msg = self.bridge.cv2_to_imgmsg(cam_img, encoding="bgr8")
        out_msg.header = msg.header

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
