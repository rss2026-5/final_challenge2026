#!/usr/bin/env python3
"""Click-to-publish goal helper for race day.

RViz's 'Publish Point' button publishes geometry_msgs/PointStamped on
/clicked_point. overall_controller listens to PoseArray on
/exploring_challenge. This node bridges them: collects N clicks
(default 2), then publishes them as a single latched PoseArray.

Usage on the car:
  ros2 run final_challenge rviz_goal_publisher
  -- in RViz, click 'Publish Point' twice on the two parking locations
  -- the FSM picks them up via the latched topic

Re-running this node lets you set fresh goals without re-launching the
full stack.
"""
import rclpy
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class RvizGoalPublisher(Node):
    def __init__(self):
        super().__init__("rviz_goal_publisher")

        self.declare_parameter("num_goals", 2)
        self.declare_parameter("output_topic", "/exploring_challenge")
        self.declare_parameter("frame_id", "map")
        self.num_goals = int(self.get_parameter("num_goals").value)
        out_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        # TRANSIENT_LOCAL so a late-starting overall_controller still sees the goals.
        self.qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.points = []
        self.pub = self.create_publisher(PoseArray, out_topic, self.qos)
        self.sub = self.create_subscription(
            PointStamped, "/clicked_point", self._on_click, 10
        )

        self.get_logger().info(
            f"Click 'Publish Point' in RViz {self.num_goals} times to set goals. "
            f"Will publish PoseArray on {out_topic}."
        )

    def _on_click(self, msg: PointStamped):
        self.points.append((msg.point.x, msg.point.y))
        idx = len(self.points)
        self.get_logger().info(
            f"Goal {idx}/{self.num_goals}: ({msg.point.x:.2f}, {msg.point.y:.2f})"
        )
        if len(self.points) >= self.num_goals:
            self._publish_goals()

    def _publish_goals(self):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.frame_id
        for x, y in self.points:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            # Yaw 0 (orientation w=1.0). FSM only uses position.
            p.orientation.w = 1.0
            arr.poses.append(p)
        self.pub.publish(arr)
        self.get_logger().info(f"Published {len(arr.poses)} goals to FSM. Re-run this node to send a new set.")
        # Reset so user can click again to send a fresh batch.
        self.points = []


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
