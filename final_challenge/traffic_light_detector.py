#!/usr/bin/env python3
"""Red traffic-light detector for Part B.

Subscribes to the ZED RGB stream, isolates red blobs in the upper portion of
the image with two HSV ranges (red wraps the hue circle), keeps only roughly
circular components, and publishes Bool on /traffic_light/state. Hysteresis on
both edges so a single noisy frame can't flip the FSM. The README says we only
need to detect the red light — we don't need to detect green to resume.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__("traffic_light_detector")

        self.declare_parameter("camera_topic", "/zed/rgb/image_rect_color")
        self.declare_parameter("state_topic", "/traffic_light/state")
        self.declare_parameter("debug_image_topic", "/traffic_light/debug_image")
        # Red wraps hue=0, so we mask two ranges and OR them.
        self.declare_parameter("hsv_lower_1", [0, 120, 100])
        self.declare_parameter("hsv_upper_1", [10, 255, 255])
        self.declare_parameter("hsv_lower_2", [170, 120, 100])
        self.declare_parameter("hsv_upper_2", [180, 255, 255])
        # Lights mount high; ignore the lower portion of the frame.
        self.declare_parameter("roi_top_fraction", 0.0)
        self.declare_parameter("roi_bottom_fraction", 0.6)
        self.declare_parameter("min_area_px", 80)
        self.declare_parameter("min_circularity", 0.6)
        self.declare_parameter("frames_to_set_red", 3)
        self.declare_parameter("frames_to_clear_red", 5)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.debug_topic = self.get_parameter("debug_image_topic").value
        self.lower1 = np.array(self.get_parameter("hsv_lower_1").value, dtype=np.uint8)
        self.upper1 = np.array(self.get_parameter("hsv_upper_1").value, dtype=np.uint8)
        self.lower2 = np.array(self.get_parameter("hsv_lower_2").value, dtype=np.uint8)
        self.upper2 = np.array(self.get_parameter("hsv_upper_2").value, dtype=np.uint8)
        self.roi_top = float(self.get_parameter("roi_top_fraction").value)
        self.roi_bottom = float(self.get_parameter("roi_bottom_fraction").value)
        self.min_area = int(self.get_parameter("min_area_px").value)
        self.min_circularity = float(self.get_parameter("min_circularity").value)
        self.frames_to_set = int(self.get_parameter("frames_to_set_red").value)
        self.frames_to_clear = int(self.get_parameter("frames_to_clear_red").value)

        self.bridge = CvBridge()
        self.red_hits = 0
        self.clear_hits = 0
        self.is_red = False

        self.state_pub = self.create_publisher(Bool, self.state_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.on_image, 1
        )

        # Latch initial state to False so downstream sees a value early.
        self._publish_state(False, force=True)

        self.get_logger().info(
            f"TrafficLightDetector ready | camera={self.camera_topic} "
            f"roi=[{self.roi_top:.2f},{self.roi_bottom:.2f}] "
            f"min_area={self.min_area} min_circ={self.min_circularity}"
        )

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failure: {e}")
            return

        h, w = frame.shape[:2]
        r0 = int(self.roi_top * h)
        r1 = int(self.roi_bottom * h)
        if r1 <= r0:
            return

        roi = frame[r0:r1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1) | cv2.inRange(
            hsv, self.lower2, self.upper2
        )

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        red_blob = self._find_red_blob(mask)

        if red_blob is not None:
            self.red_hits += 1
            self.clear_hits = 0
        else:
            self.clear_hits += 1
            self.red_hits = 0

        if not self.is_red and self.red_hits >= self.frames_to_set:
            self.is_red = True
            self._publish_state(True)
            self.get_logger().info("Red light detected — publishing True")
        elif self.is_red and self.clear_hits >= self.frames_to_clear:
            self.is_red = False
            self._publish_state(False)
            self.get_logger().info("Red light cleared — publishing False")

        self._publish_debug(frame, red_blob, r0, msg.header)

    def _find_red_blob(self, mask):
        """Return (cx, cy, area, circularity, contour) of best red blob, or None."""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best = None
        best_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            perim = cv2.arcLength(c, True)
            if perim <= 0:
                continue
            circularity = 4.0 * np.pi * area / (perim * perim)
            if circularity < self.min_circularity:
                continue
            if area > best_area:
                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                best = (cx, cy, area, circularity, c)
                best_area = area
        return best

    def _publish_state(self, is_red, force=False):
        msg = Bool()
        msg.data = bool(is_red)
        self.state_pub.publish(msg)

    def _publish_debug(self, frame, red_blob, roi_offset, header):
        out = frame.copy()
        h = out.shape[0]
        # Show ROI band.
        cv2.rectangle(
            out,
            (0, int(self.roi_top * h)),
            (out.shape[1] - 1, int(self.roi_bottom * h)),
            (255, 255, 0),
            1,
        )
        if red_blob is not None:
            cx, cy, area, circ, contour = red_blob
            shifted = contour.copy()
            shifted[:, 0, 1] += roi_offset
            cv2.drawContours(out, [shifted], -1, (0, 0, 255), 2)
            cv2.putText(
                out,
                f"red a={int(area)} c={circ:.2f}",
                (cx, cy + roi_offset - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
            )
        status_color = (0, 0, 255) if self.is_red else (0, 255, 0)
        status_text = "RED" if self.is_red else "clear"
        cv2.putText(
            out,
            f"state: {status_text} (r={self.red_hits} c={self.clear_hits})",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
        )
        try:
            out_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            out_msg.header = header
            self.debug_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
