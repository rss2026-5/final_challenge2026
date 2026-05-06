#!/usr/bin/env python3
"""Drives the car the last ~1m to the parking meter using its
homography-projected position.

The TA-published goal pose is approximate — only good for getting the car
to the cluster of three roadside objects. The 1m-in-front-of-meter
parking criterion is measured against the actual meter, so we close the
loop using the perception output.

Subscribes:
  /parking_meter/relative_position (PointStamped, base_link, x forward,
    y left, meters): position of the chosen meter from object_detector.
    Published every frame a meter is in view.
  /object_detection (String): 'parking_meter' once persistence
    confirms a stable detection. We use this as the gate to start
    approaching — only after object_detector has confirmed.

Publishes:
  /vesc/input/navigation (or whatever drive_topic param says): drive
    commands while approaching.
  /parking_approach/complete (Bool, latched True): set True when the car
    is within stop_distance of the meter for a sustained window.
    overall_controller listens for this to advance to PARKING state.

Lifecycle:
  - IDLE: no String received yet OR no position seen recently. Don't
    publish drive.
  - APPROACHING: String fired AND position is fresh. Drive toward the
    meter. Slow down near the goal. Publish complete=True when within
    stop_distance for stop_hold_sec consecutive seconds.
  - DONE: complete fired. Stay silent on drive (FSM owns from here).
    Reset on next /object_detection event so this works for both meters.
"""
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ParkingApproach(Node):
    def __init__(self):
        super().__init__("parking_approach")

        self.declare_parameter("drive_topic", "/vesc/input/navigation")
        self.declare_parameter("position_topic", "/parking_meter/relative_position")
        self.declare_parameter("detection_topic", "/object_detection")
        self.declare_parameter("complete_topic", "/parking_approach/complete")
        # Target stop distance from the meter (meters, in front of car).
        self.declare_parameter("stop_distance", 1.0)
        # Tolerance window around stop_distance — how close is "close enough."
        self.declare_parameter("stop_tolerance", 0.15)
        # Hold within tolerance for this long before signalling complete.
        self.declare_parameter("stop_hold_sec", 0.4)
        # Position freshness — if no message in this many seconds, stop driving.
        self.declare_parameter("position_timeout_sec", 0.5)
        # Speed limits.
        self.declare_parameter("max_speed", 0.7)
        self.declare_parameter("min_speed", 0.25)
        # Steering gain (radians per meter of lateral offset). Bounded by max_steer.
        self.declare_parameter("steer_gain", 1.2)
        self.declare_parameter("max_steer", 0.34)
        # Distance at which we start tapering speed toward min_speed.
        self.declare_parameter("slow_distance", 1.8)

        self.drive_topic = self.get_parameter("drive_topic").value
        self.position_topic = self.get_parameter("position_topic").value
        self.detection_topic = self.get_parameter("detection_topic").value
        self.complete_topic = self.get_parameter("complete_topic").value
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.stop_tolerance = float(self.get_parameter("stop_tolerance").value)
        self.stop_hold_sec = float(self.get_parameter("stop_hold_sec").value)
        self.position_timeout_sec = float(
            self.get_parameter("position_timeout_sec").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.steer_gain = float(self.get_parameter("steer_gain").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)

        # State.
        # active=True after /object_detection fires, until we publish
        # complete (or until a new detection rearms us for the next meter).
        self.active = False
        self.complete_published = False
        self.last_position = None  # (x, y, ros_time) in base_link
        self.in_window_since = None  # ros_time first entered tolerance band

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.complete_pub = self.create_publisher(
            Bool, self.complete_topic, 10)

        self.create_subscription(
            PointStamped, self.position_topic, self.position_cb, 10)
        self.create_subscription(
            String, self.detection_topic, self.detection_cb, 10)

        # 20Hz control loop, mirrors the FSM cadence.
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"ParkingApproach ready | stop_distance={self.stop_distance}m "
            f"tolerance={self.stop_tolerance}m max_speed={self.max_speed}m/s "
            f"position_topic={self.position_topic}"
        )

    def detection_cb(self, msg: String):
        if "parking_meter" not in msg.data.lower():
            return
        # Re-arm for the new meter. complete_published gets reset so we'll
        # fire a fresh complete when we close on this one.
        self.active = True
        self.complete_published = False
        self.in_window_since = None
        self.get_logger().info(
            "Detection received — approaching meter (active=True)"
        )

    def position_cb(self, msg: PointStamped):
        x = float(msg.point.x)
        y = float(msg.point.y)
        self.last_position = (x, y, self.get_clock().now())

    def control_loop(self):
        if not self.active:
            return
        if self.complete_published:
            # Done with this meter — FSM is in PARKING / dwell. Don't fight it.
            return
        if self.last_position is None:
            return

        x, y, ts = self.last_position
        age = (self.get_clock().now() - ts).nanoseconds * 1e-9
        if age > self.position_timeout_sec:
            # Lost sight of the meter — stop and wait. If it's gone for
            # the whole detection window, the FSM's timeout will fire.
            self._publish_drive(0.0, 0.0)
            self.in_window_since = None
            return

        distance = math.hypot(x, y)
        # Distance error: how much further forward we still need to go.
        # Positive = drive forward, negative = we're past the meter.
        error = distance - self.stop_distance

        # Tolerance band check — if close enough, hold and start the dwell.
        if abs(error) <= self.stop_tolerance:
            now = self.get_clock().now()
            if self.in_window_since is None:
                self.in_window_since = now
                self._publish_drive(0.0, 0.0)
                return
            held = (now - self.in_window_since).nanoseconds * 1e-9
            self._publish_drive(0.0, 0.0)
            if held >= self.stop_hold_sec:
                self._publish_complete()
            return

        # Outside the tolerance band — left it for some reason, reset.
        self.in_window_since = None

        # Speed: taper from max -> min as distance shrinks toward stop_distance.
        if distance >= self.slow_distance:
            speed_mag = self.max_speed
        else:
            # Linear taper between slow_distance and stop_distance.
            denom = max(self.slow_distance - self.stop_distance, 1e-3)
            t = (distance - self.stop_distance) / denom
            t = max(0.0, min(1.0, t))
            speed_mag = self.min_speed + t * (self.max_speed - self.min_speed)

        # If we overshot, back up at min speed.
        if error < 0:
            speed = -self.min_speed
        else:
            speed = speed_mag

        # Steer toward the meter laterally.
        # atan2(y, x) gives angle from car heading to meter.
        # Positive y = meter is to the left; positive steering = left turn.
        if distance > 1e-3:
            target_angle = math.atan2(y, x)
        else:
            target_angle = 0.0
        steer = max(
            -self.max_steer,
            min(self.max_steer, self.steer_gain * target_angle),
        )

        self._publish_drive(speed, steer)

    def _publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def _publish_complete(self):
        if self.complete_published:
            return
        self.complete_published = True
        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)
        self.get_logger().info(
            "Approach complete — within stop_distance for stop_hold_sec"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParkingApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
