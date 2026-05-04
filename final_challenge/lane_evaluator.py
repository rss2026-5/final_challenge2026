#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import numpy as np
import math

class LaneEvaluator(Node):
    def __init__(self):
        super().__init__('lane_evaluator')
        
        # Subscriber for Lane Error (Float32)
        self.error_sub = self.create_subscription(
            Float32, '/lane_error', self.error_callback, 10)
        
        # Subscriber for Lane Target (PointStamped)
        self.target_sub = self.create_subscription(
            PointStamped, '/lane_target', self.target_callback, 10)

        # Metrics Accumulators
        self.errors = []
        self.movements = []
        self.total_frames = 0
        self.detected_frames = 0
        self.prev_point = None

    def error_callback(self, msg):
        self.total_frames += 1
        # Use -1.0 or NaN to signal no detection in your pipeline
        if not np.isnan(msg.data) and msg.data != -1.0:
            self.detected_frames += 1
            self.errors.append(msg.data)
            
        if self.total_frames % 100 == 0:
            self.report()

    def target_callback(self, msg):
        current_point = msg.point
        
        if self.prev_point is not None:
            # Calculate Euclidean Distance (Hypotenuse) between current and last point
            dist = math.sqrt((current_point.x - self.prev_point.x)**2 + 
                             (current_point.y - self.prev_point.y)**2)
            self.movements.append(dist)

        self.prev_point = current_point

    def report(self):
        # 1. Calculate Confidence
        confidence = (self.detected_frames / self.total_frames) * 100 if self.total_frames > 0 else 0
        
        # 2. Calculate Jitter (Std Dev of error)
        # Higher values mean the lane line detection is flickering
        jitter = np.std(self.errors[-100:]) if len(self.errors) > 1 else 0
        
        # 3. Calculate Mean Movement (Target Displacement)
        # Higher values mean the target point is "jumping" erratically
        mean_move = np.mean(self.movements[-100:]) if len(self.movements) > 1 else 0
        
        self.get_logger().info(
            f"\n--- LANE PERFORMANCE REPORT ---\n"
            f"Frames Processed: {self.total_frames}\n"
            f"Confidence:      {confidence:.2f}%\n"
            f"Center Jitter:   {jitter:.2f} px (RMS)\n"
            f"Target Movement: {mean_move:.2f} units/frame\n"
            f"Avg Lane Error:  {np.mean(self.errors):.2f} px\n"
            f"-------------------------------"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LaneEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()