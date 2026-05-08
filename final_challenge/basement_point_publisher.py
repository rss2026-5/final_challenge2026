#!/usr/bin/env python3
"""Stand-in for the course's basement_point_publisher.

Publishes a fixed PoseArray of goal locations (read from params) on
/exploring_challenge so overall_controller can leave IDLE in sim.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose


class BasementPointPublisher(Node):
    def __init__(self):
        super().__init__("basement_point_publisher")

        self.declare_parameter("topic", "/exploring_challenge")
        self.declare_parameter("frame_id", "map")
        # Flat list [x1, y1, yaw1, x2, y2, yaw2, ...]. Yaw optional (0 by default).
        self.declare_parameter("goals", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("startup_delay", 2.0)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        flat = list(self.get_parameter("goals").value)
        delay = float(self.get_parameter("startup_delay").value)

        if len(flat) % 3 != 0 or len(flat) == 0:
            self.get_logger().error(
                f"'goals' must be a non-empty flat list of [x, y, yaw] triples, got {flat}"
            )
            flat = []
        self.goals = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self.pub = self.create_publisher(PoseArray, topic, 10)
        self._timer = self.create_timer(delay, self._publish_once)
        self.get_logger().info(
            f"Will publish {len(self.goals)} goals to {topic} in {delay:.1f}s"
        )

    def _publish_once(self):
        self._timer.cancel()
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y, yaw in self.goals:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            half = float(yaw) * 0.5
            p.orientation.z = math.sin(half)
            p.orientation.w = math.cos(half)
            msg.poses.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f"Published {len(msg.poses)} goals")


def main(args=None):
    rclpy.init(args=args)
    node = BasementPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
