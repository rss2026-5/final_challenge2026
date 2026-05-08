#!/usr/bin/env bash
#
# Push final_challenge to the racecar via rsync.
#
# Usage:
#   ./scripts/deploy.sh           # defaults to car 102
#   ./scripts/deploy.sh 104       # override car number
#
# After this finishes, SSH into the car, run `connect`, then:
#   cd ~/racecar_ws && colcon build --symlink-install --packages-select final_challenge
#   source install/setup.bash

set -euo pipefail

CAR="${1:-102}"
ROBOT="racecar@192.168.1.${CAR}"
REMOTE_DIR="~/racecar_ws/src/final_challenge"
LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Deploying ${LOCAL_ROOT} -> ${ROBOT}:${REMOTE_DIR}"

rsync -av --delete \
    --exclude '.git' \
    --exclude '__pycache__' \
    --exclude '.claude' \
    --exclude '*.pyc' \
    --exclude 'media/briefing' \
    --exclude 'racetrack_images' \
    "${LOCAL_ROOT}/" \
    "${ROBOT}:${REMOTE_DIR}/"

echo ""
echo "==> Done. Now on the car (after \`connect\`):"
echo ""
echo "    cd ~/racecar_ws && colcon build --symlink-install --packages-select final_challenge"
echo "    source install/setup.bash"
echo ""
echo "==> Launch options:"
echo "    ros2 launch final_challenge race_real.launch.xml      # Part A"
echo "    ros2 launch final_challenge boating_real.launch.xml   # Part B"
