#!/usr/bin/env bash
set -euo pipefail

patterns=(
  'ros2 launch drone_bringup full_stack.launch.py'
  'launch_px4_instance.sh 0 x500_d1 d1'
  'launch_px4_instance.sh 1 x500_d2 d2'
  'MicroXRCEAgent udp4 -p 8888'
  'MicroXRCEAgent udp4 -p 8889'
  'gz sim -r -v4 -s --headless-rendering .*/worlds/maze(_ceiling)?\.sdf'
  'gcs_heartbeat.py'
  'bootstrap_ekf2.py'
  'auto_save_arena_map.py'
  'parameter_bridge --ros-args --log-level warn'
  'lio_sam_(imuPreintegration|imageProjection|featureExtraction|mapOptimization)'
  'octomap_server_node'
  'frontier_detector'
  'offboard_controller'
  'exploration_planner'
  'drone_coordinator'
)

echo "[cleanup] stopping full-stack processes"
for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 2
for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done

echo "[cleanup] remaining matches:"
pgrep -af "$(IFS='|'; echo "${patterns[*]}")" || true
