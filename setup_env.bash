#!/bin/bash

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1. Source ROS2 workspace (if it exists)
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
    echo " Sourced ROS2 Workspace"
else
    echo "  Warning: install/setup.bash not found. Run 'colcon build' first."
fi

# 2. PX4 Paths
# Modify this to the actual path of your PX4-Autopilot directory
export PX4_DIR=~/px4_workspace/PX4-Autopilot

# 3. Gazebo & PX4 Config
export PX4_GZ_MODEL_NAME=x500_vision_lidar
export PX4_GZ_WORLD=maze

# Logging + Gazebo home (keep writable inside repo)
export GZ_HOMEDIR="$REPO_ROOT/.gz"
export ROS_LOG_DIR="$REPO_ROOT/.ros/log"
mkdir -p "$GZ_HOMEDIR" "$ROS_LOG_DIR"

# Add your repo's models and worlds to Gazebo's path
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/models:$REPO_ROOT/worlds:$GZ_SIM_RESOURCE_PATH"

# Point to your startup parameters file
export PX4_STARTUP_SCRIPT="$REPO_ROOT/init/params.pks"

echo " PX4 Environment Ready: $PX4_GZ_MODEL_NAME in $PX4_GZ_WORLD"
echo " To launch: cd \$PX4_DIR && make px4_sitl gz_x500 PX4_CONFIG_FILE=\$PX4_STARTUP_SCRIPT"
