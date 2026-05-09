#!/usr/bin/env bash
# Bundle every diagnostic we usually run for boating-launch issues into
# one paste-and-forget output. Run after `connect` and `source
# ~/racecar_ws/install/setup.bash`.
#
# Usage:
#   bash ~/racecar_ws/src/final_challenge/scripts/diagnose.sh

echo "=================================================="
echo "NODES"
echo "=================================================="
ros2 node list
echo

echo "=================================================="
echo "DUPLICATE NODE NAMES (anything appearing twice is a bug)"
echo "=================================================="
ros2 node list 2>/dev/null | sort | uniq -c | awk '$1 > 1'
echo

echo "=================================================="
echo "TOPICS (boating-relevant)"
echo "=================================================="
ros2 topic list | grep -E '/map|/scan|/odom|/pf|/initialpose|/object_detect|/parking|/traffic_light|/exploring_challenge|/clicked_point|/goal_pose|/vesc/input|/drive' | sort
echo

echo "=================================================="
echo "/initialpose pub/sub counts"
echo "=================================================="
ros2 topic info /initialpose
echo

echo "=================================================="
echo "/map pub/sub counts"
echo "=================================================="
ros2 topic info /map
echo

echo "=================================================="
echo "/scan pub/sub counts + rate"
echo "=================================================="
ros2 topic info /scan
timeout 3 ros2 topic hz /scan 2>&1 | head -5
echo

echo "=================================================="
echo "/pf/pose/odom pub/sub counts + rate"
echo "=================================================="
ros2 topic info /pf/pose/odom
timeout 3 ros2 topic hz /pf/pose/odom 2>&1 | head -5
echo

echo "=================================================="
echo "/parking_meter/relative_position pub/sub counts + rate"
echo "=================================================="
ros2 topic info /parking_meter/relative_position 2>&1
timeout 3 ros2 topic hz /parking_meter/relative_position 2>&1 | head -3
echo

echo "=================================================="
echo "/vesc/input/navigation publishers (who's fighting for the wheel?)"
echo "=================================================="
ros2 topic info /vesc/input/navigation --verbose 2>&1 | grep -E 'Node name|Publisher count|Subscription count'
echo

echo "=================================================="
echo "/vesc/high_level/input/nav_2 publishers (the new drive topic)"
echo "=================================================="
ros2 topic info /vesc/high_level/input/nav_2 --verbose 2>&1 | grep -E 'Node name|Publisher count|Subscription count'
echo

echo "=================================================="
echo "TF: does map -> base_link exist?"
echo "=================================================="
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -8
echo

echo "=================================================="
echo "TF: does odom -> base_link exist?"
echo "=================================================="
timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8
echo

echo "=================================================="
echo "ALL TF FRAMES"
echo "=================================================="
timeout 3 ros2 run tf2_tools view_frames 2>&1 | tail -5
echo

echo "=================================================="
echo "PROCESS CHECK: anything related to localization / particle filter"
echo "=================================================="
ps -ef | grep -iE 'particle|localization|pf|map_server' | grep -v grep
echo

echo "=================================================="
echo "PROCESS CHECK: trajectory_follower (likely fighting drive topic)"
echo "=================================================="
ps -ef | grep -iE 'trajectory_follower|parking_approach|overall_controller' | grep -v grep
echo

echo "=================================================="
echo "DONE — paste everything above back to Claude."
echo "=================================================="
