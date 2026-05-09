#!/usr/bin/env bash
# Race-day one-shot startup. Run inside docker after `connect`.
#
# Brings up teleop + ZED + boating stack. RViz and rviz_goal_publisher
# stay manual (open in noVNC). Logs go to /tmp/race_logs/ so we can
# tail them after the run for postmortem.
#
# Usage:
#   bash ~/racecar_ws/src/final_challenge/scripts/race_day_start.sh
#
# To stop everything cleanly:
#   bash ~/racecar_ws/src/final_challenge/scripts/race_day_stop.sh

set -o pipefail

# colcon's setup.bash references some optional unbound vars, so don't `set -u`
source ~/racecar_ws/install/setup.bash

LOG_DIR=/tmp/race_logs
mkdir -p "$LOG_DIR"

# Fail fast if anything's still running from a previous attempt.
EXISTING=$(ps -ef | grep -E 'particle_filter|overall_controller|object_detector|traffic_light_detector|parking_approach|trajectory_planner|trajectory_follower|safety_controller|urg_node|vesc_driver|zed_node|rrt_planner' | grep -v grep | wc -l)
if [ "$EXISTING" -gt 0 ]; then
    echo "WARNING: existing race processes detected. Run race_day_stop.sh first, or:"
    echo "  pkill -f boating_real; pkill -f teleop; pkill -f zed_camera"
    echo ""
    ps -ef | grep -E 'particle_filter|overall_controller|object_detector|traffic_light_detector|parking_approach|trajectory_planner|trajectory_follower|safety_controller|urg_node|vesc_driver|zed_node|rrt_planner' | grep -v grep
    exit 1
fi

echo "==> 1/3 Starting teleop (VESC, joy, lidar, static TFs)..."
ros2 launch racecar teleop.launch.xml > "$LOG_DIR/teleop.log" 2>&1 &
TELEOP_PID=$!
echo "    pid=$TELEOP_PID, log=$LOG_DIR/teleop.log"
sleep 8

echo "==> 2/3 Starting ZED camera..."
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed > "$LOG_DIR/zed.log" 2>&1 &
ZED_PID=$!
echo "    pid=$ZED_PID, log=$LOG_DIR/zed.log"
sleep 10

echo "==> 3/3 Starting boating stack (PF, planner, follower, perception, FSM, parking_approach)..."
ros2 launch final_challenge boating_real.launch.xml > "$LOG_DIR/boating.log" 2>&1 &
BOATING_PID=$!
echo "    pid=$BOATING_PID, log=$LOG_DIR/boating.log"
sleep 12

echo ""
echo "============================================================"
echo "==> Sanity check"
echo "============================================================"

echo ""
echo "--- Active nodes (should see particle_filter, overall_controller, etc.) ---"
ros2 node list 2>/dev/null | grep -E 'particle_filter|overall_controller|object_detector|traffic_light|parking_approach|trajectory_planner|trajectory_follower|safety_controller|urg_node|vesc_driver|zed_node' | sort

echo ""
echo "--- /scan rate (lidar; expect ~10-40 Hz) ---"
timeout 3 ros2 topic hz /scan 2>&1 | head -3

echo ""
echo "--- /vesc/sensors/core rate (VESC; expect ~50 Hz) ---"
timeout 3 ros2 topic hz /vesc/sensors/core 2>&1 | head -3

echo ""
echo "--- /zed/zed_node/rgb/image_rect_color rate (camera; expect ~28 Hz) ---"
timeout 3 ros2 topic hz /zed/zed_node/rgb/image_rect_color 2>&1 | head -3

echo ""
echo "--- /pf/pose/odom rate (PF; will be 0 until you click 2D Pose Estimate) ---"
timeout 3 ros2 topic hz /pf/pose/odom 2>&1 | head -3

echo ""
echo "--- /initialpose subscribers (must be 1+; PF must be listening) ---"
ros2 topic info /initialpose | grep count

echo ""
echo "--- recent boating launch errors (should be empty) ---"
grep -iE 'error|fail|exception|traceback|died' "$LOG_DIR/boating.log" 2>/dev/null | tail -5

echo ""
echo "============================================================"
echo "==> Stack is up. Next steps:"
echo "============================================================"
echo ""
echo "  In another SSH terminal (after connect + source):"
echo "    ros2 run final_challenge rviz_goal_publisher"
echo ""
echo "  In the noVNC desktop:"
echo "    rviz2 -d ~/racecar_ws/install/final_challenge/share/final_challenge/launch/real/boating.rviz"
echo ""
echo "  In RViz:"
echo "    1. Click '2D Pose Estimate', click on map at car location, drag direction."
echo "    2. Click 'Publish Point', click first parking goal."
echo "    3. Click 'Publish Point', click second parking goal."
echo "    4. Watch the boating launch log: tail -f $LOG_DIR/boating.log"
echo ""
echo "  PIDs: teleop=$TELEOP_PID  zed=$ZED_PID  boating=$BOATING_PID"
echo "  Stop everything: bash ~/racecar_ws/src/final_challenge/scripts/race_day_stop.sh"
