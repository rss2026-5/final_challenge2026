#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class State(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    DETECTING = "DETECTING"
    PARKING = "PARKING"
    RETURNING = "RETURNING"
    DONE = "DONE"


class GoalType(Enum):
    PARK = "PARK"   # stop, detect, park 5s, then advance
    PASS = "PASS"   # drive through with loose tolerance, then advance


class OverallController(Node):
    def __init__(self):
        super().__init__("overall_controller")

        # Parameters
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("goal_arrival_threshold", 1.0)
        self.declare_parameter("pass_arrival_threshold", 2.0)
        self.declare_parameter("parking_duration", 5.0)
        self.declare_parameter("detection_window", 3.0)
        #[x, y, yaw] waypoints as intermediate steps to hardcode long loop back.
        #appended to the queue after the last PARK completes.
        self.declare_parameter("return_waypoints", [0.0])
        #Small wait time to let particle filter converge, so initial pose is stable
        self.declare_parameter("startup_delay_sec", 5.0)
        #Saves initial pose (after delay) as position to ultimately return to
        self.declare_parameter("auto_return_to_start", True)

        odom_topic = self.get_parameter("odom_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        self.goal_arrival_threshold = self.get_parameter("goal_arrival_threshold").value
        self.pass_arrival_threshold = self.get_parameter("pass_arrival_threshold").value
        self.parking_duration = self.get_parameter("parking_duration").value
        self.detection_window = self.get_parameter("detection_window").value
        self.return_waypoints = self._parse_waypoints(
            list(self.get_parameter("return_waypoints").value)
        )
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.auto_return_to_start = bool(self.get_parameter("auto_return_to_start").value)
        self._return_leg_appended = False
        self._localization_ready = False
        self._pending_goals = None
        self.start_pose = None

        # State
        self.state = State.IDLE
        # (PoseStamped, GoalType) entries
        self.goal_queue = []
        self.current_goal_index = 0
        self.current_pose = None
        self.red_light_detected = False
        self.meter_detected = False
        self.detection_timer = None
        self.parking_timer = None

        # Subscribers
        self.goals_sub = self.create_subscription(
            PoseArray, "/exploring_challenge", self.goals_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)
        self.detection_sub = self.create_subscription(
            String, "/object_detection", self.object_detection_cb, 10)
        self.traffic_sub = self.create_subscription(
            Bool, "/traffic_light/state", self.traffic_light_cb, 10)
        # parking_approach drives the last ~1m using the meter's homography
        # position. It signals here when the car is parked correctly and
        # ready for the 5s dwell.
        self.approach_sub = self.create_subscription(
            Bool, "/parking_approach/complete",
            self.approach_complete_cb, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.follower_enable_pub = self.create_publisher(Bool, "/trajectory_follower/enabled", 10)

        # 20Hz control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)

        #Startup timer for particle filter
        self._startup_timer = self.create_timer(
            self.startup_delay_sec, self._on_startup_complete
        )

        # 1Hz heartbeat with state + localization + goal progress
        self._first_odom_logged = False
        self._first_traffic_logged = False
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat)

        self.get_logger().info(
            f"OverallController initialized; waiting {self.startup_delay_sec:.1f}s "
            "for localization before accepting goals..."
            f" (odom_topic={odom_topic}, drive_topic={drive_topic})"
        )

    def goals_cb(self, msg: PoseArray):
        """Receive goal sequence from course staff. Accepted at any time —
        on race day we may need to re-issue goals after a manual abort
        without relaunching the whole stack.
        """
        self.get_logger().info(
            f"[goals_cb] received {len(msg.poses)} goal(s) "
            f"(state={self.state.value}, localization_ready={self._localization_ready})"
        )

        if not self._localization_ready:
            #Process upon localization warmup's conclusion
            self._pending_goals = msg
            self.get_logger().info(
                "[goals_cb] localization warmup not finished; deferring "
                f"({len(msg.poses)} goals queued)."
            )
            return

        # Hard reset the FSM. Any in-flight detection/parking timers get
        # cancelled, queue replaced, return-leg flag cleared. This makes
        # mid-run re-issuing of goals safe.
        if self.state != State.IDLE:
            self.get_logger().info(
                f"[goals_cb] resetting from {self.state.value} to ingest new goals"
            )
            if self.detection_timer is not None:
                self.detection_timer.cancel()
                self.detection_timer = None
            if self.parking_timer is not None:
                self.parking_timer.cancel()
                self.parking_timer = None
            self._return_leg_appended = False
            self.state = State.IDLE

        self._ingest_goals(msg)

    def _ingest_goals(self, msg: PoseArray):
        self.goal_queue = []
        for pose in msg.poses:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose = pose
            self.goal_queue.append((goal, GoalType.PARK))

        if len(self.goal_queue) == 0:
            self.get_logger().warn("Received empty goal list")
            return

        self.get_logger().info(f"Received {len(self.goal_queue)} parking goals")
        self.current_goal_index = 0
        self._publish_current_goal()
        self._transition_to(State.NAVIGATING)

    def odom_cb(self, msg: Odometry):
        """Update current pose. Check goal arrival when NAVIGATING."""
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        self.current_pose = (pos.x, pos.y, yaw)
        if not self._first_odom_logged:
            self._first_odom_logged = True
            self.get_logger().info(
                f"[odom_cb] first odom received at "
                f"({pos.x:.2f}, {pos.y:.2f}, yaw={yaw:.2f})"
            )

        if self.state == State.NAVIGATING:
            dist = self._distance_to_current_goal()
            goal_type = self._current_goal_type()
            threshold = (
                self.pass_arrival_threshold
                if goal_type == GoalType.PASS
                else self.goal_arrival_threshold
            )
            if dist < threshold:
                self.get_logger().info(
                    f"Arrived at goal {self.current_goal_index} "
                    f"({goal_type.value}, dist={dist:.2f}m)"
                )
                if goal_type == GoalType.PASS:
                    self._transition_to(State.RETURNING)
                else:
                    self._transition_to(State.DETECTING)

    def object_detection_cb(self, msg: String):
        """YOLO detection results. Only act on them during DETECTING state."""
        if self.state != State.DETECTING:
            return

        if "parking_meter" in msg.data.lower():
            self.get_logger().info(
                "Parking meter detected — handing off to parking_approach"
            )
            self.meter_detected = True
            # Don't transition to PARKING yet; parking_approach will drive
            # the last ~1m and tell us via /parking_approach/complete.

    def approach_complete_cb(self, msg: Bool):
        """parking_approach has the car within stop_distance — start dwell."""
        if self.state != State.DETECTING or not msg.data:
            return
        self.get_logger().info(
            "Parking approach complete — starting 5s dwell"
        )
        if self.detection_timer is not None:
            self.detection_timer.cancel()
            self.detection_timer = None
        self._transition_to(State.PARKING)

    def traffic_light_cb(self, msg: Bool):
        """Update red light flag. The control_loop handles stopping."""
        if not self._first_traffic_logged:
            self._first_traffic_logged = True
            self.get_logger().info(
                f"[traffic_light_cb] first message received: red={msg.data}"
            )
        if msg.data != self.red_light_detected:
            self.get_logger().info(
                f"[traffic_light_cb] state change: red={msg.data} (state={self.state.value})"
            )
        self.red_light_detected = msg.data

    def control_loop(self):
        """Runs at 20Hz. Publishes stop commands when the car should be stationary.

        Note: in DETECTING, parking_approach owns the drive topic — we
        deliberately don't publish stop here, otherwise we'd fight it on
        the same topic at 20 Hz. PARKING/DONE we still own.
        """
        if self.state == State.NAVIGATING and self.red_light_detected:
            self._publish_stop()
        elif self.state in (State.PARKING, State.DONE):
            self._publish_stop()

    def _set_follower_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.follower_enable_pub.publish(msg)

    def _transition_to(self, new_state: State):
        """Handle entry actions for each state."""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"State: {old_state.value} -> {new_state.value}")

        if new_state == State.NAVIGATING:
            self._set_follower_enabled(True)

        elif new_state == State.DETECTING:
            self._set_follower_enabled(False)
            self.meter_detected = False
            self.detection_timer = self.create_timer(
                self.detection_window, self._detection_timeout_cb)

        elif new_state == State.PARKING:
            self._set_follower_enabled(False)
            self.parking_timer = self.create_timer(
                self.parking_duration, self._parking_complete_cb)

        elif new_state == State.RETURNING:
            self._set_follower_enabled(True)
            self._advance_to_next_goal()

        elif new_state == State.DONE:
            self._set_follower_enabled(False)
            self.get_logger().info("All goals visited. Challenge complete.")

    def _on_startup_complete(self):
        """One-time after startup_delay_sec. Snapshots start pose and processes goals."""
        self._startup_timer.cancel()
        self._startup_timer = None
        self._localization_ready = True
        self.get_logger().info(
            "[startup] localization warmup elapsed — accepting goals "
            f"(time-based gate; not a PF-convergence check). odom_seen={self.current_pose is not None}, "
            f"pending_goals={'yes' if self._pending_goals is not None else 'no'}"
        )

        if self.current_pose is not None:
            self.start_pose = self.current_pose
            self.get_logger().info(
                f"Localization warmup complete. Start pose snapshot: "
                f"({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f}, "
                f"yaw={self.start_pose[2]:.2f})"
            )
        else:
            self.get_logger().warn(
                "Localization warmup elapsed but no odom received yet — "
                "auto return-to-start will not be available."
            )

        if self._pending_goals is not None:
            pending = self._pending_goals
            self._pending_goals = None
            self._ingest_goals(pending)

    def _detection_timeout_cb(self):
        """Detection window expired without finding a parking meter."""
        self.detection_timer.cancel()
        self.detection_timer = None
        if not self.meter_detected:
            self.get_logger().info(
                f"No parking meter at goal {self.current_goal_index}, moving on")
            self._transition_to(State.RETURNING)

    def _parking_complete_cb(self):
        """5-second parking dwell complete."""
        self.parking_timer.cancel()
        self.parking_timer = None
        self.get_logger().info(
            f"Parking complete at goal {self.current_goal_index}")
        self._transition_to(State.RETURNING)

    def _advance_to_next_goal(self):
        """Move to next goal in queue, or finish if none remain."""
        self.current_goal_index += 1

        #checks/adds final leg to return to starting position
        if not self._return_leg_appended and self.current_goal_index >= len(self.goal_queue):
            return_pts = list(self.return_waypoints)
            # snapshotted start pose as the final return target.
            if self.auto_return_to_start and self.start_pose is not None:
                return_pts.append(self.start_pose)
            if return_pts:
                self._return_leg_appended = True
                for (x, y, yaw) in return_pts:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = float(x)
                    pose.pose.position.y = float(y)
                    half = float(yaw) * 0.5
                    pose.pose.orientation.z = math.sin(half)
                    pose.pose.orientation.w = math.cos(half)
                    self.goal_queue.append((pose, GoalType.PASS))
                self.get_logger().info(
                    f"Appended {len(return_pts)} return waypoint(s) "
                    f"(auto_return_to_start={self.auto_return_to_start})"
                )
        if self.current_goal_index < len(self.goal_queue):
            self._publish_current_goal()
            self._transition_to(State.NAVIGATING)
        else:
            self._transition_to(State.DONE)

    def _publish_current_goal(self):
        """Send current goal to trajectory_planner via /goal_pose."""
        goal, goal_type = self.goal_queue[self.current_goal_index]
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)
        pos = goal.pose.position
        cur = self.current_pose
        cur_str = (
            f"from ({cur[0]:.2f}, {cur[1]:.2f})"
            if cur is not None else "from <no odom yet>"
        )
        self.get_logger().info(
            f"[publish_goal] -> /goal_pose idx={self.current_goal_index}/"
            f"{len(self.goal_queue)} [{goal_type.value}] "
            f"target=({pos.x:.2f}, {pos.y:.2f}) {cur_str}"
        )

    def _publish_stop(self):
        """Publish zero-velocity drive command to stop the car."""
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _distance_to_current_goal(self):
        """Euclidean distance from current pose to current goal."""
        if self.current_pose is None or self.current_goal_index >= len(self.goal_queue):
            return float('inf')
        goal_pos = self.goal_queue[self.current_goal_index][0].pose.position
        dx = self.current_pose[0] - goal_pos.x
        dy = self.current_pose[1] - goal_pos.y
        return np.sqrt(dx * dx + dy * dy)

    def _current_goal_type(self):
        if self.current_goal_index >= len(self.goal_queue):
            return GoalType.PARK
        return self.goal_queue[self.current_goal_index][1]

    def _heartbeat(self):
        """1Hz status line covering state, localization, goal progress."""
        odom_str = (
            f"pose=({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f})"
            if self.current_pose is not None else "pose=<none>"
        )
        if self.goal_queue and self.current_goal_index < len(self.goal_queue):
            dist = self._distance_to_current_goal()
            goal_str = (
                f"goal={self.current_goal_index}/{len(self.goal_queue)} "
                f"dist={dist:.2f}m"
            )
        elif self.goal_queue:
            goal_str = f"goal=done ({len(self.goal_queue)} total)"
        elif self._pending_goals is not None:
            goal_str = f"goal=pending ({len(self._pending_goals.poses)} deferred)"
        else:
            goal_str = "goal=<none>"
        self.get_logger().info(
            f"[heartbeat] state={self.state.value} "
            f"loc_ready={self._localization_ready} {odom_str} {goal_str} "
            f"red={self.red_light_detected}"
        )

    @staticmethod
    def _parse_waypoints(flat):
        """Turn a flat [x, y, yaw, ...] param into [(x, y, yaw), ...]. Empty if malformed."""
        if len(flat) < 3 or len(flat) % 3 != 0:
            return []
        # The default we set is [0.0], a placeholder for "none". Real lists are >= 3.
        return [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]


def main(args=None):
    rclpy.init(args=args)
    node = OverallController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
