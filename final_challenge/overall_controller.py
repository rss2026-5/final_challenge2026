#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
import tf_transformations


class State(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    DETECTING = "DETECTING"
    PARKING = "PARKING"
    RETURNING = "RETURNING"
    DONE = "DONE"


class OverallController(Node):
    def __init__(self):
        super().__init__("overall_controller")

        # Parameters
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("goal_arrival_threshold", 1.0)
        self.declare_parameter("parking_duration", 5.0)
        self.declare_parameter("detection_window", 3.0)

        odom_topic = self.get_parameter("odom_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        self.goal_arrival_threshold = self.get_parameter("goal_arrival_threshold").value
        self.parking_duration = self.get_parameter("parking_duration").value
        self.detection_window = self.get_parameter("detection_window").value

        # State
        self.state = State.IDLE
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

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # 20Hz control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("OverallController initialized, waiting for goals...")

    def goals_cb(self, msg: PoseArray):
        """Receive goal sequence from course staff. Only processed once in IDLE."""
        if self.state != State.IDLE:
            return

        self.goal_queue = []
        for pose in msg.poses:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose = pose
            self.goal_queue.append(goal)

        if len(self.goal_queue) == 0:
            self.get_logger().warn("Received empty goal list")
            return

        self.get_logger().info(f"Received {len(self.goal_queue)} goals")
        self.current_goal_index = 0
        self._publish_current_goal()
        self._transition_to(State.NAVIGATING)

    def odom_cb(self, msg: Odometry):
        """Update current pose. Check goal arrival when NAVIGATING."""
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        self.current_pose = (pos.x, pos.y, yaw)

        if self.state == State.NAVIGATING:
            dist = self._distance_to_current_goal()
            if dist < self.goal_arrival_threshold:
                self.get_logger().info(
                    f"Arrived at goal {self.current_goal_index} (dist={dist:.2f}m)")
                self._transition_to(State.DETECTING)

    def object_detection_cb(self, msg: String):
        """YOLO detection results. Only act on them during DETECTING state."""
        if self.state != State.DETECTING:
            return

        if "parking_meter" in msg.data.lower():
            self.get_logger().info("Parking meter detected!")
            self.meter_detected = True
            if self.detection_timer is not None:
                self.detection_timer.cancel()
                self.detection_timer = None
            self._transition_to(State.PARKING)

    def traffic_light_cb(self, msg: Bool):
        """Update red light flag. The control_loop handles stopping."""
        self.red_light_detected = msg.data

    def control_loop(self):
        """Runs at 20Hz. Publishes stop commands when the car should be stationary."""
        if self.state == State.NAVIGATING and self.red_light_detected:
            self._publish_stop()
        elif self.state in (State.DETECTING, State.PARKING, State.DONE):
            self._publish_stop()

    def _transition_to(self, new_state: State):
        """Handle entry actions for each state."""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"State: {old_state.value} -> {new_state.value}")

        if new_state == State.DETECTING:
            self.meter_detected = False
            self.detection_timer = self.create_timer(
                self.detection_window, self._detection_timeout_cb)

        elif new_state == State.PARKING:
            self.parking_timer = self.create_timer(
                self.parking_duration, self._parking_complete_cb)

        elif new_state == State.RETURNING:
            self._advance_to_next_goal()

        elif new_state == State.DONE:
            self.get_logger().info("All goals visited. Challenge complete.")

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
        if self.current_goal_index < len(self.goal_queue):
            self._publish_current_goal()
            self._transition_to(State.NAVIGATING)
        else:
            self._transition_to(State.DONE)

    def _publish_current_goal(self):
        """Send current goal to trajectory_planner via /goal_pose."""
        goal = self.goal_queue[self.current_goal_index]
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)
        pos = goal.pose.position
        self.get_logger().info(
            f"Published goal {self.current_goal_index}: ({pos.x:.2f}, {pos.y:.2f})")

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
        goal_pos = self.goal_queue[self.current_goal_index].pose.position
        dx = self.current_pose[0] - goal_pos.x
        dy = self.current_pose[1] - goal_pos.y
        return np.sqrt(dx * dx + dy * dy)


def main(args=None):
    rclpy.init(args=args)
    node = OverallController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
