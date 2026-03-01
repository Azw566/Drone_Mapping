#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/setup_env.bash"

export GZ_HOMEDIR="$REPO_ROOT/.gz"
export ROS_LOG_DIR="$REPO_ROOT/.ros/log"
mkdir -p "$GZ_HOMEDIR" "$ROS_LOG_DIR"

ros2 launch drone_bringup arm_takeoff_test.launch.py use_rviz:=false
