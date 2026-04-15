#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped


class OverallController(Node):
    def __init__(self):
        super().__init__("overall_controller")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = OverallController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
