#!/usr/bin/env python3
"""Fakes environmental stimuli for Part B sim.

Without cameras, neither the YOLO object detector nor the HSV traffic-light
detector will ever fire. This node watches /odom and injects:

- /object_detection (String) "parking_meter" when the car enters a radius
  around a configured goal. A per-goal probability lets us also exercise
  the detection-timeout path.
- /traffic_light/state (Bool) True briefly when the car enters a radius
  around the traffic-light location.

A latched `pedestrian_present` param is accepted but intentionally
unimplemented in v1 — see plan.
"""
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class SimEnvironment(Node):
    def __init__(self):
        super().__init__("sim_environment")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("object_detection_topic", "/object_detection")
        self.declare_parameter("traffic_light_topic", "/traffic_light/state")
        self.declare_parameter("approach_complete_topic", "/parking_approach/complete")
        # Delay between fake meter detection and fake approach-complete, so
        # the FSM's NAVIGATING -> DETECTING -> PARKING transitions are
        # observable in logs/rviz before we jump to the dwell.
        self.declare_parameter("approach_complete_delay_sec", 1.0)

        # Flat [x1, y1, x2, y2, ...]; must match goals in basement_point_publisher.
        self.declare_parameter("goal_points", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("goal_detection_radius", 1.5)
        # Per-goal probability that the meter will be "detected" (else timeout path).
        self.declare_parameter("goal_detection_probabilities", [1.0, 1.0])

        self.declare_parameter("simulate_red_light", True)
        self.declare_parameter("traffic_light_point", [0.0, 0.0])
        self.declare_parameter("traffic_light_radius", 2.5)
        self.declare_parameter("red_light_duration", 3.0)
        # If > 0, light starts RED for this many seconds after launch regardless
        # of car position (for end-to-end testing of the red-light-stop path).
        self.declare_parameter("startup_red_duration", 0.0)

        self.declare_parameter("pedestrian_present", False)

        odom_topic = self.get_parameter("odom_topic").value
        obj_topic = self.get_parameter("object_detection_topic").value
        tl_topic = self.get_parameter("traffic_light_topic").value
        ac_topic = self.get_parameter("approach_complete_topic").value
        self.approach_complete_delay = float(
            self.get_parameter("approach_complete_delay_sec").value)

        gp = list(self.get_parameter("goal_points").value)
        self.goals = [(gp[i], gp[i + 1]) for i in range(0, len(gp), 2)]
        self.goal_radius = float(self.get_parameter("goal_detection_radius").value)
        probs = list(self.get_parameter("goal_detection_probabilities").value)
        if len(probs) < len(self.goals):
            probs = list(probs) + [1.0] * (len(self.goals) - len(probs))
        self.goal_probs = probs
        self.goal_fired = [False] * len(self.goals)

        self.simulate_red = bool(self.get_parameter("simulate_red_light").value)
        tl = list(self.get_parameter("traffic_light_point").value)
        self.tl_point = (tl[0], tl[1]) if len(tl) >= 2 else (0.0, 0.0)
        self.tl_radius = float(self.get_parameter("traffic_light_radius").value)
        self.red_duration = float(self.get_parameter("red_light_duration").value)
        self.tl_triggered = False
        self.tl_active = False
        self._tl_timer = None

        if bool(self.get_parameter("pedestrian_present").value):
            self.get_logger().warn(
                "pedestrian_present=True but pedestrian injection is not "
                "implemented in this node yet — ignoring"
            )

        self.obj_pub = self.create_publisher(String, obj_topic, 10)
        # Fake parking_approach output so the FSM's new approach-complete
        # path also works in sim (no real perception/approach controller).
        self.approach_pub = self.create_publisher(Bool, ac_topic, 10)
        # Only create a traffic-light publisher if this node will actually
        # drive the light (startup or proximity). Otherwise we leave the
        # topic entirely to external publishers (e.g. manual ros2 topic pub,
        # or the real traffic_light_detector on hardware).
        startup_red = float(self.get_parameter("startup_red_duration").value)
        self._tl_auto = self.simulate_red or startup_red > 0.0
        self.tl_pub = (
            self.create_publisher(Bool, tl_topic, 10) if self._tl_auto else None
        )
        # Subscribe so the marker color mirrors whatever anyone publishes.
        self.tl_sub = self.create_subscription(Bool, tl_topic, self._tl_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, "/sim_environment/markers", 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # Marker refresh (colors track the last /traffic_light/state message).
        self.create_timer(0.5, self._publish_markers)

        if startup_red > 0.0:
            self.tl_triggered = True  # suppress proximity trigger entirely
            self.tl_active = True
            self.get_logger().info(
                f"Sim: traffic light RED at startup, will go GREEN in {startup_red:.1f}s"
            )
            msg_out = Bool()
            msg_out.data = True
            self.tl_pub.publish(msg_out)
            self._tl_timer = self.create_timer(startup_red, self._clear_red)

        self.get_logger().info(
            f"SimEnvironment: {len(self.goals)} goals (r={self.goal_radius}m), "
            f"auto_traffic_light={'on' if self._tl_auto else 'off (external only)'}"
        )

    def _tl_cb(self, msg: Bool):
        # Track state from any publisher so the marker shows the right color.
        self.tl_active = bool(msg.data)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        for i, (gx, gy) in enumerate(self.goals):
            if self.goal_fired[i]:
                continue
            if (x - gx) ** 2 + (y - gy) ** 2 <= self.goal_radius ** 2:
                self.goal_fired[i] = True
                if random.random() < self.goal_probs[i]:
                    self.get_logger().info(f"Sim: announcing parking_meter at goal {i}")
                    out = String()
                    out.data = "parking_meter"
                    self.obj_pub.publish(out)
                    # Fake the approach-complete signal after a short delay
                    # so the FSM advances DETECTING -> PARKING.
                    self.create_timer(
                        self.approach_complete_delay,
                        self._fake_approach_complete,
                    )
                else:
                    self.get_logger().info(
                        f"Sim: skipping detection at goal {i} (probability roll)"
                    )

        if self.simulate_red and not self.tl_triggered:
            tx, ty = self.tl_point
            if (x - tx) ** 2 + (y - ty) ** 2 <= self.tl_radius ** 2:
                self.tl_triggered = True
                self.tl_active = True
                self.get_logger().info("Sim: red light ON")
                msg_out = Bool()
                msg_out.data = True
                self.tl_pub.publish(msg_out)
                self._tl_timer = self.create_timer(self.red_duration, self._clear_red)

    def _fake_approach_complete(self):
        # FSM only acts on this in DETECTING; once it transitions to
        # PARKING it ignores further messages, so re-firing is harmless.
        msg = Bool()
        msg.data = True
        self.approach_pub.publish(msg)
        self.get_logger().info("Sim: faking parking_approach/complete=True")

    def _clear_red(self):
        self._tl_timer.cancel()
        self._tl_timer = None
        self.tl_active = False
        self.get_logger().info("Sim: traffic light -> GREEN")
        msg_out = Bool()
        msg_out.data = False
        self.tl_pub.publish(msg_out)


    def _publish_markers(self):
        arr = MarkerArray()

        # Traffic light: sphere at tl_point, colored by state.
        tl = Marker()
        tl.header.frame_id = "map"
        tl.header.stamp = self.get_clock().now().to_msg()
        tl.ns = "traffic_light"
        tl.id = 0
        tl.type = Marker.SPHERE
        tl.action = Marker.ADD
        tl.pose.position.x = float(self.tl_point[0])
        tl.pose.position.y = float(self.tl_point[1])
        tl.pose.position.z = 0.5
        tl.pose.orientation.w = 1.0
        tl.scale.x = tl.scale.y = tl.scale.z = 0.6
        if self.tl_active:
            tl.color.r, tl.color.g, tl.color.b = 1.0, 0.0, 0.0
        else:
            tl.color.r, tl.color.g, tl.color.b = 0.0, 1.0, 0.0
        tl.color.a = 1.0
        arr.markers.append(tl)

        # Translucent cylinder showing the proximity trigger radius.
        ring = Marker()
        ring.header.frame_id = "map"
        ring.header.stamp = self.get_clock().now().to_msg()
        ring.ns = "traffic_light"
        ring.id = 1
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position.x = float(self.tl_point[0])
        ring.pose.position.y = float(self.tl_point[1])
        ring.pose.position.z = 0.05
        ring.pose.orientation.w = 1.0
        ring.scale.x = ring.scale.y = 2.0 * self.tl_radius
        ring.scale.z = 0.05
        ring.color.r, ring.color.g, ring.color.b = 1.0, 0.8, 0.0
        ring.color.a = 0.25
        arr.markers.append(ring)

        # Goal pillars + text labels.
        for i, (gx, gy) in enumerate(self.goals):
            pillar = Marker()
            pillar.header.frame_id = "map"
            pillar.header.stamp = self.get_clock().now().to_msg()
            pillar.ns = "goals"
            pillar.id = i * 2
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.pose.position.x = float(gx)
            pillar.pose.position.y = float(gy)
            pillar.pose.position.z = 0.75
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = pillar.scale.y = 0.4
            pillar.scale.z = 1.5
            pillar.color.r, pillar.color.g, pillar.color.b = 0.2, 0.4, 1.0
            pillar.color.a = 1.0
            arr.markers.append(pillar)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "goals"
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(gx)
            label.pose.position.y = float(gy)
            label.pose.position.z = 2.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.8
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"goal {i + 1}"
            arr.markers.append(label)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SimEnvironment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
