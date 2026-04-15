#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
