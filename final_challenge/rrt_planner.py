#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from scipy.ndimage import binary_dilation
from .utils import LineTrajectory


class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__("trajectory_planner")
        pass


def main(args=None):
    rclpy.init(args=args)
    planner = RRTStarPlanner()
    rclpy.spin(planner)
    rclpy.shutdown()
