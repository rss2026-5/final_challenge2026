#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class RaceSafetyController(Node):
    """
    Narrow forward-only safety stop for racing.
    Only monitors a tight cone ahead — deliberately ignores the sides
    so adjacent-lane cars don't trigger a false stop.
    Uses TTC so stopping distance scales with speed.
    """

    FORWARD_CONE     = np.pi / 8  # ±22.5° — tight, only things directly ahead
    MIN_DIST         = 0.3        # absolute floor [m]
    TTC_THRESHOLD    = 0.5        # seconds — stop if dist/speed < this
    RELEASE_MARGIN   = 0.2        # hysteresis [m]
    RANGE_PERCENTILE = 10         # 10th percentile to ignore noisy returns

    def __init__(self):
        super().__init__("race_safety_controller")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("nav_topic", "/drive")

        scan_topic  = self.get_parameter("scan_topic").get_parameter_value().string_value
        drive_topic = self.get_parameter("drive_topic").get_parameter_value().string_value
        nav_topic   = self.get_parameter("nav_topic").get_parameter_value().string_value

        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.create_subscription(AckermannDriveStamped, nav_topic, self.drive_cb, 10)
        self.safety_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.current_speed = 0.0
        self.in_emergency  = False

        self.get_logger().info(
            f"Race safety controller ready — scan: {scan_topic}, "
            f"safety: {drive_topic}, nav: {nav_topic}"
        )

    def drive_cb(self, msg: AckermannDriveStamped):
        self.current_speed = msg.drive.speed

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        fwd_mask = np.isfinite(ranges) & (ranges > 0.0) & (np.abs(angles) < self.FORWARD_CONE)
        if not fwd_mask.any():
            return

        front_dist = float(np.percentile(ranges[fwd_mask], self.RANGE_PERCENTILE))
        threshold  = max(self.MIN_DIST, abs(self.current_speed) * self.TTC_THRESHOLD)

        if not self.in_emergency and front_dist < threshold:
            self.in_emergency = True
            self.get_logger().warn(
                f"STOP: obstacle {front_dist:.2f}m < {threshold:.2f}m "
                f"(speed={self.current_speed:.1f} m/s)"
            )
        elif self.in_emergency and front_dist > threshold + self.RELEASE_MARGIN:
            self.in_emergency = False
            self.get_logger().info(f"Obstacle cleared at {front_dist:.2f}m, resuming")

        if self.in_emergency:
            stop = AckermannDriveStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = "base_link"
            stop.drive.speed = 0.0
            stop.drive.steering_angle = 0.0
            self.safety_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = RaceSafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
