#!/usr/bin/env python3
"""Parking-meter detector for Part B.

Subscribes to the ZED RGB stream, runs YOLO inference, picks the most likely
parking meter when multiple candidates fire, and publishes a String on
/object_detection that the overall_controller substring-matches against
"parking_meter". Also saves the annotated frame each time a detection is
announced so we have a record for grading per the README.
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
import torch
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


# Homography calibration carried over from Lab 4. Re-calibrate later if needed.
PTS_IMAGE_PLANE = [
    [403, 286],
    [203, 280],
    [378, 249],
    [569, 279],
    [366, 195],
]
PTS_GROUND_PLANE = [
    [10, 0],
    [10, 10],
    [15, 0],
    [10, -10],
    [35, 0],
]
METERS_PER_INCH = 0.0254

PARKING_METER_CLASS = "parking meter"


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("camera_topic", "/zed/rgb/image_rect_color")
        self.declare_parameter("detection_topic", "/object_detection")
        self.declare_parameter("debug_image_topic", "/object_detector/debug_image")
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.7)
        self.declare_parameter("persistence_frames", 3)
        self.declare_parameter("min_aspect_ratio", 1.5)
        self.declare_parameter("max_aspect_ratio", 6.0)
        self.declare_parameter("save_dir", os.path.expanduser("~/parking_captures"))
        self.declare_parameter("republish_cooldown_sec", 8.0)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.debug_topic = self.get_parameter("debug_image_topic").value
        self.model_name = self.get_parameter("model").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.persistence_frames = int(self.get_parameter("persistence_frames").value)
        self.min_aspect = float(self.get_parameter("min_aspect_ratio").value)
        self.max_aspect = float(self.get_parameter("max_aspect_ratio").value)
        self.save_dir = self.get_parameter("save_dir").value
        self.cooldown = float(self.get_parameter("republish_cooldown_sec").value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(self.model_name)
        self.model.to(self.device)

        self.meter_class_id = self._resolve_class_id(PARKING_METER_CLASS)
        if self.meter_class_id is None:
            self.get_logger().error(
                f"Model {self.model_name} has no '{PARKING_METER_CLASS}' class — "
                "object_detector will never fire."
            )

        # Image -> ground homography (camera frame, x forward, y left, meters).
        ground = np.array(PTS_GROUND_PLANE, dtype=np.float32) * METERS_PER_INCH
        image = np.array(PTS_IMAGE_PLANE, dtype=np.float32)
        self.h_matrix, _ = cv2.findHomography(
            image[:, np.newaxis, :], ground[:, np.newaxis, :]
        )

        self.bridge = CvBridge()
        self.consecutive_hits = 0
        self.last_publish_time = None
        self.detection_count = 0

        self.det_pub = self.create_publisher(String, self.detection_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.on_image, 1
        )

        self.get_logger().info(
            f"ObjectDetector ready | model={self.model_name} device={self.device} "
            f"camera={self.camera_topic} class_id={self.meter_class_id} "
            f"persistence={self.persistence_frames}"
        )

    def _resolve_class_id(self, target_name):
        for cid, name in self.model.names.items():
            if name.lower() == target_name.lower():
                return int(cid)
        return None

    def on_image(self, msg: Image):
        if self.meter_class_id is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failure: {e}")
            return

        try:
            results = self.model(
                frame,
                classes=[self.meter_class_id],
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        candidates = self._extract_candidates(results, frame.shape[:2])
        chosen = self._select_meter(candidates)

        annotated = self._draw_overlay(frame, candidates, chosen)
        self._publish_debug(annotated, msg.header)

        if chosen is None:
            self.consecutive_hits = 0
            return

        self.consecutive_hits += 1
        if self.consecutive_hits < self.persistence_frames:
            return

        if self._in_cooldown():
            return

        self._announce(annotated)

    def _extract_candidates(self, results, img_shape):
        """Filter YOLO output to viable parking-meter candidates."""
        if not results or results[0].boxes is None:
            return []

        boxes = results[0].boxes
        xyxy = boxes.xyxy.detach().cpu().numpy()
        conf = boxes.conf.detach().cpu().numpy()
        cls = boxes.cls.detach().cpu().numpy().astype(int)

        h, w = img_shape
        out = []
        for box, c, k in zip(xyxy, conf, cls):
            if k != self.meter_class_id:
                continue
            x1, y1, x2, y2 = box
            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)
            aspect = bh / bw  # parking meters are tall, so h/w >> 1
            if aspect < self.min_aspect or aspect > self.max_aspect:
                continue
            ground_xy = self._project_bottom_center(x1, y1, x2, y2)
            if ground_xy is None or ground_xy[0] <= 0:
                continue
            out.append(
                {
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "conf": float(c),
                    "aspect": float(aspect),
                    "ground": ground_xy,
                    "distance": float(np.hypot(ground_xy[0], ground_xy[1])),
                }
            )
        return out

    def _project_bottom_center(self, x1, y1, x2, y2):
        u = (x1 + x2) / 2.0
        v = y2  # bottom edge sits on the ground plane
        pt = np.array([[u], [v], [1.0]], dtype=np.float64)
        proj = self.h_matrix @ pt
        if abs(proj[2, 0]) < 1e-9:
            return None
        proj /= proj[2, 0]
        return float(proj[0, 0]), float(proj[1, 0])

    def _select_meter(self, candidates):
        """Closest-forward candidate wins. Heuristic, no goal-pose info used."""
        if not candidates:
            return None
        return min(candidates, key=lambda c: c["distance"])

    def _in_cooldown(self):
        if self.last_publish_time is None:
            return False
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
        return elapsed < self.cooldown

    def _announce(self, annotated):
        msg = String()
        msg.data = "parking_meter"
        self.det_pub.publish(msg)
        self.last_publish_time = self.get_clock().now()
        self.consecutive_hits = 0
        self.detection_count += 1
        path = self._save_image(annotated)
        self.get_logger().info(
            f"Published parking_meter detection #{self.detection_count} | saved {path}"
        )

    def _save_image(self, image):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"parking_meter_{self.detection_count:02d}_{stamp}.png"
        path = os.path.join(self.save_dir, filename)
        try:
            cv2.imwrite(path, image)
        except Exception as e:
            self.get_logger().error(f"Failed to save {path}: {e}")
            return None
        return path

    def _draw_overlay(self, frame, candidates, chosen):
        out = frame.copy()
        for cand in candidates:
            x1, y1, x2, y2 = cand["bbox"]
            color = (0, 255, 0) if cand is chosen else (0, 165, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = (
                f"meter {cand['conf']:.2f} "
                f"d={cand['distance']:.2f}m"
            )
            cv2.putText(
                out, label, (x1, max(0, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )
        if chosen is not None:
            cv2.putText(
                out,
                f"hits {self.consecutive_hits}/{self.persistence_frames}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
        return out

    def _publish_debug(self, image, header):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            out_msg.header = header
            self.debug_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
