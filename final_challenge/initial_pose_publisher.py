#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("publish_period", 0.5)

        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.yaw = self.get_parameter("yaw").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        self.remaining = int(self.get_parameter("publish_count").value)
        period = float(self.get_parameter("publish_period").value)

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, latched)

        self._timer = self.create_timer(period, self._tick)
        self._done = False
        self.get_logger().info(
            f"Will publish initial pose ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f} rad) "
            f"to {topic} {self.remaining} times"
        )

    def _tick(self):
        if self._done:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        half = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.remaining -= 1
        if self.remaining <= 0:
            self._done = True
            self._timer.cancel()
            self.get_logger().info("Initial pose broadcast complete")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
