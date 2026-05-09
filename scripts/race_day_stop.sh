#!/usr/bin/env bash
# Cleanly stop everything race_day_start.sh brought up.
# Usage: bash ~/racecar_ws/src/final_challenge/scripts/race_day_stop.sh

echo "==> Stopping race-day stack..."

# Kill the launch wrappers first (lets them clean up children gracefully)
pkill -f 'ros2 launch racecar teleop' 2>/dev/null
pkill -f 'ros2 launch zed_wrapper' 2>/dev/null
pkill -f 'ros2 launch final_challenge boating_real' 2>/dev/null
sleep 2

# Then nuke any stragglers by node-pattern
pkill -f boating_real 2>/dev/null
pkill -f particle_filter 2>/dev/null
pkill -f trajectory_follower 2>/dev/null
pkill -f trajectory_planner 2>/dev/null
pkill -f rrt_planner 2>/dev/null
pkill -f parking_approach 2>/dev/null
pkill -f overall_controller 2>/dev/null
pkill -f object_detector 2>/dev/null
pkill -f traffic_light_detector 2>/dev/null
pkill -f safety_controller 2>/dev/null
pkill -f rviz_goal_publisher 2>/dev/null
pkill -f map_server 2>/dev/null
pkill -f lifecycle_manager 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f throttle_interpolator 2>/dev/null
pkill -f ackermann_to_vesc 2>/dev/null
pkill -f vesc_to_odom 2>/dev/null
pkill -f mux_chainer 2>/dev/null
pkill -f vesc_driver 2>/dev/null
pkill -f joy_teleop 2>/dev/null
pkill -f joy_node 2>/dev/null
pkill -f ackermann_cmd 2>/dev/null
pkill -f urg_node2 2>/dev/null
pkill -f zed_node 2>/dev/null
pkill -f zed_container 2>/dev/null
sleep 2

# Verify
LEFTOVER=$(ps -ef | grep -E 'particle_filter|overall_controller|object_detector|traffic_light_detector|parking_approach|trajectory_planner|trajectory_follower|rrt_planner|safety_controller|urg_node|vesc_driver|zed_node|joy_teleop' | grep -v grep | wc -l)
if [ "$LEFTOVER" -gt 0 ]; then
    echo "==> Some processes remain:"
    ps -ef | grep -E 'particle_filter|overall_controller|object_detector|traffic_light_detector|parking_approach|trajectory_planner|trajectory_follower|rrt_planner|safety_controller|urg_node|vesc_driver|zed_node|joy_teleop' | grep -v grep
else
    echo "==> Clean. Ready for race_day_start.sh."
fi
