#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, PointStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from final_challenge.detect_lane_lines import detect_lane_lines


class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter("lookahead_ratio", 0.6)
        self.lookahead_ratio = self.get_parameter("lookahead_ratio").value

        self.declare_parameter("smoothing_alpha", 0.2)
        self.alpha = self.get_parameter("smoothing_alpha").value

        self.prev_error = 0.0

        # --- Subscribers ---
        self.sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        # --- Publishers ---
        self.debug_pub = self.create_publisher(Image, "/lane_debug", 10)
        self.error_pub = self.create_publisher(Float32, "/lane_error", 10)
        self.target_pub = self.create_publisher(PointStamped, "/lane_target", 10)


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_h, img_w = frame.shape[:2]

        # --- Run detection ---
        detection_output = detect_lane_lines(frame, self.lookahead_ratio)
        output_img = detection_output["image"]
        left_line = detection_output["left_line"]
        right_line = detection_output["right_line"]

        # --- Lookahead position ---
        y_eval = int(img_h * self.lookahead_ratio)

        def get_x_at_y(line, y):
            if line is None:
                return None
            vx, vy, x0, y0 = line
            if abs(vy) < 1e-6:
                return None
            return int(x0 + (y - y0) * vx / vy)

        left_x = get_x_at_y(left_line, y_eval)
        right_x = get_x_at_y(right_line, y_eval)

        lane_center = None
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) // 2
        elif left_x is not None:
            lane_center = left_x + 100  # fallback (approx half lane width)
        elif right_x is not None:
            lane_center = right_x - 100

        # --- If no detection ---
        if lane_center is None:
            self.get_logger().warn("Lane not detected")
            return

        # --- Compute lateral error ---
        img_center = img_w // 2
        error = (lane_center - img_center) / img_center  # normalize [-1, 1]

        # --- Smooth error ---
        error = self.alpha * error + (1 - self.alpha) * self.prev_error
        self.prev_error = error

        # --- Publish lateral error ---
        error_msg = Float32()
        error_msg.data = float(error)
        self.error_pub.publish(error_msg)

        # --- Publish lookahead target ---
        target_msg = PointStamped()
        target_msg.header = msg.header
        target_msg.point.x = float(lane_center)
        target_msg.point.y = float(y_eval)
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)

        # --- Visualization ---
        cv2.circle(output_img, (lane_center, y_eval), 6, (0, 255, 255), -1)
        cv2.line(output_img, (img_center, img_h), (lane_center, y_eval), (0, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(output_img, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
