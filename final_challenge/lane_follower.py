#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")

        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.declare_parameter("line_follower", True)

        self.get_logger().info(f"Drive topic: {self.drive_topic}")
        self.create_subscription(
            PointStamped, "/irl_lane_target", self.target_callback, 1)
        
        self.parking_distance = 0.1  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.speed = 4.0 # hardcoded max speed
        self.acceptable_distance_error = self.speed * 0.01 # can change this
        self.acceptable_angle_error = 0.05 # can change this
        self.max_angle = 0.4 # can change this
        self.previous_steering_angle = None

        self.get_logger().info("Lane Follower Initialized")
    
    def target_callback(self, msg):
        self.relative_x = msg.point.x
        self.relative_y = msg.point.y
        drive_cmd = AckermannDriveStamped()

        relative_distance = np.sqrt(self.relative_x ** 2 + self.relative_y ** 2)
        distance_error = relative_distance - self.parking_distance
        relative_angle = np.arctan2(self.relative_y, self.relative_x)
        steering_angle = min(np.abs(relative_angle), self.max_angle) * (relative_angle / np.abs(relative_angle))

        velocity = self.speed
        if np.abs(relative_angle) < self.max_angle: # if target is within front cone
            if np.abs(distance_error) < self.acceptable_distance_error and np.abs(relative_angle) < self.acceptable_angle_error:
                # stop if both distance and angle errors are acceptable
                velocity *= 0
            elif distance_error < 0:
                # back up if target is too close in front
                velocity *= -1
            else:
                # go forward if target is far in front
                velocity *= 1
            # slow down when close to target:
            if np.abs(distance_error) < self.acceptable_distance_error * 5.0:
                velocity *= np.abs(distance_error) / (self.acceptable_distance_error * 5.0)
        elif np.abs(relative_angle) > np.pi - self.max_angle: # if target is within back cone
            # defaults to driving foward, causing the car to circle until the target is within the front cone
            velocity *= 1
        else:
            if relative_distance < self.speed * 0.5 / np.sin(self.max_angle):
                # back up if target is in "unreachable zones"
                velocity *= -1
                steering_angle = 0.0
            else: # if target is far to the side
                # defaults to driving foward, causing the car to circle until the target is within the front cone
                velocity *= 1

        """
        # Line follower: never reverse, always drive forward
        if self.line_follower and velocity < 0:
            velocity = self.speed * 0.5
        """
        self.get_logger().info(f"Publishing drive: {velocity}")
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.drive.speed = velocity
        drive_cmd.drive.steering_angle = steering_angle

        self.drive_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
