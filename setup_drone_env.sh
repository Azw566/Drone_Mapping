#!/bin/bash
# =============================================================================
# Drone Mapping Full Environment Setup
# Ubuntu 22.04 (WSL2 or native)
# Stack: ROS2 Humble + Gazebo Harmonic + PX4 v1.16 SITL + Drone_Mapping
# =============================================================================
set -e

echo "=========================================="
echo " Drone Mapping Full Environment Setup"
echo "=========================================="

# ── Step 1: Base build tools ───────────────────────────────────────────────
echo "[1/8] Installing base build tools..."
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  build-essential cmake ninja-build git curl wget \
  python3-pip python3-dev python3-setuptools python3-wheel \
  lsb-release gnupg2 software-properties-common

# ── Step 2: ROS2 Humble ────────────────────────────────────────────────────
echo "[2/8] Installing ROS2 Humble..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-desktop \
  python3-rosdep \
  python3-colcon-common-extensions

rosdep init 2>/dev/null || true
rosdep update

# ── Step 3: Gazebo Harmonic ────────────────────────────────────────────────
echo "[3/8] Installing Gazebo Harmonic..."
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y gz-harmonic

# ── Step 4: ROS2 packages + system deps ───────────────────────────────────
echo "[4/8] Installing ROS2 packages and system dependencies..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-perception-pcl \
  ros-humble-pcl-ros \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-octomap-ros \
  ros-humble-octomap-server \
  ros-humble-nav2-msgs \
  ros-humble-visualization-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-opencv \
  libopencv-contrib-dev \
  astyle ccache clang llvm gcc g++ gdb make rsync \
  shellcheck unzip xsltproc zip genromfs bc \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
  libpcl-dev libeigen3-dev libboost-all-dev libxml2-utils

# GTSAM (from PPA — not in default Ubuntu repo)
add-apt-repository -y ppa:borglab/gtsam-release-4.1
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  libgtsam-dev libgtsam-unstable-dev

# Python packages
pip3 install -q \
  numpy scipy matplotlib transforms3d pyserial \
  empy==3.3.4 packaging jinja2 pyyaml jsonschema \
  toml pyros-genmsg colcon-common-extensions

# ── Step 5: px4_custom ────────────────────────────────────────────────────
echo "[5/8] Cloning and building px4_custom..."
mkdir -p ~/px4_workspace
git clone --recursive https://github.com/Azw566/px4_custom.git ~/px4_workspace

PX4_DIR=~/px4_workspace/PX4-Autopilot
cd "$PX4_DIR"

# PX4 Makefile requires a .git dir — the repo was absorbed so we init one
git init -q && git add -A && git commit -q -m "init"

# Init .git in any submodule dirs that have content but no .git
grep "path = " .gitmodules | awk '{print $3}' | while read p; do
  if [ -d "$p" ] && [ ! -e "$p/.git" ]; then
    git -C "$p" init -q && git -C "$p" add -A && git -C "$p" commit -q -m "init" 2>/dev/null || true
  fi
done

# --- Fix incomplete submodules ---

# pymavlink is a nested submodule inside mavlink — was not absorbed
rm -rf src/modules/mavlink/mavlink/pymavlink
git clone --depth=1 https://github.com/ArduPilot/pymavlink.git \
  src/modules/mavlink/mavlink/pymavlink

# Micro-XRCE-DDS-Client — partially absorbed, clone fresh from px4 branch
rm -rf src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
git clone --depth=1 -b px4 https://github.com/PX4/Micro-XRCE-DDS-Client.git \
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client

# CDR streams (needed for uxrce_dds)
rm -rf src/lib/cdrstream/cyclonedds src/lib/cdrstream/rosidl
git clone --depth=1 https://github.com/px4/cyclonedds src/lib/cdrstream/cyclonedds
git clone --depth=1 https://github.com/px4/rosidl     src/lib/cdrstream/rosidl

# Gazebo Harmonic simulation models
rm -rf Tools/simulation/gz
git clone --depth=1 https://github.com/PX4/PX4-gazebo-models Tools/simulation/gz

# GPS drivers — pin to exact PX4 v1.16.0 commit to avoid API mismatch
rm -rf src/drivers/gps/devices
git clone https://github.com/PX4/PX4-GPSDrivers.git src/drivers/gps/devices
git -C src/drivers/gps/devices checkout 4cea5de87ce7e2a52e5289bd3639fd8eb770eafa

# Remaining small submodules
rm -rf src/lib/events/libevents src/lib/heatshrink/heatshrink src/lib/crypto/monocypher
git clone --depth=1 https://github.com/mavlink/libevents  src/lib/events/libevents
git clone --depth=1 https://github.com/PX4/heatshrink     src/lib/heatshrink/heatshrink
git clone --depth=1 https://github.com/PX4/Monocypher     src/lib/crypto/monocypher

# Install PX4 Python requirements
pip3 install -r Tools/setup/requirements.txt

# Build base SITL target (compiles all shared C++ — ~5-10 min on 12 cores)
make px4_sitl gz_x500

# ── Step 6: Micro-XRCE-DDS-Agent ──────────────────────────────────────────
echo "[6/8] Building Micro-XRCE-DDS-Agent..."
git clone --depth=1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  ~/Micro-XRCE-DDS-Agent
mkdir -p ~/Micro-XRCE-DDS-Agent/build
cd ~/Micro-XRCE-DDS-Agent/build
cmake .. -DCMAKE_BUILD_TYPE=Release -G Ninja
ninja -j$(nproc)

# ── Step 7: Drone_Mapping workspace ───────────────────────────────────────
echo "[7/8] Cloning and building Drone_Mapping..."
mkdir -p ~/ros_ws/drone
git clone --recurse-submodules https://github.com/Azw566/Drone_Mapping.git \
  ~/ros_ws/drone

# px4_msgs directory is empty in the repo — clone it separately at v1.16
rm -rf ~/ros_ws/drone/src/px4_msgs
git clone --depth=1 -b release/1.16 \
  https://github.com/PX4/px4_msgs.git ~/ros_ws/drone/src/px4_msgs

cd ~/ros_ws/drone
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Fix 1: Replace hardcoded /home/telemaque paths with /root
find ~/ros_ws/drone/src/ -type f \( -name "*.py" -o -name "*.sh" -o -name "*.sdf" \) \
  -exec grep -l "telemaque" {} \; | xargs sed -i 's|/home/telemaque|/root|g'
find ~/ros_ws/drone/install/ -type f \( -name "*.py" -o -name "*.sh" -o -name "*.sdf" \) \
  -exec grep -l "telemaque" {} \; | xargs sed -i 's|/home/telemaque|/root|g'

# Fix 2: Set PX4_GZ_WORLD=maze so gz_bridge subscribes to the correct
# Gazebo topic path (/world/maze/... not /world/default/...)
SCRIPT=~/ros_ws/drone/install/drone_bringup/lib/drone_bringup/launch_px4_instance.sh
sed -i 's|export PX4_GZ_STANDALONE=1|export PX4_GZ_STANDALONE=1\nexport PX4_GZ_WORLD=maze|' "$SCRIPT"
SCRIPT_SRC=~/ros_ws/drone/src/drone_bringup/scripts/launch_px4_instance.sh
sed -i 's|export PX4_GZ_STANDALONE=1|export PX4_GZ_STANDALONE=1\nexport PX4_GZ_WORLD=maze|' "$SCRIPT_SRC"

# Fix 3: Reduce physics update rate from 1000Hz to 250Hz (WSL2 clock jitter fix)
for f in ~/ros_ws/drone/src/drone_bringup/worlds/*.sdf \
          ~/ros_ws/drone/install/drone_bringup/share/drone_bringup/worlds/*.sdf; do
  sed -i 's|<real_time_update_rate>1000</real_time_update_rate>|<real_time_update_rate>250</real_time_update_rate>|g' "$f"
done

# ── Step 8: Install x500_vision_lidar model into PX4 ─────────────────────
echo "[8/8] Installing x500_vision_lidar into PX4..."
cp -r ~/ros_ws/drone/src/drone_bringup/models/x500_vision_lidar \
  ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/models/

AIRFRAMES_DIR=~/px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
cp "$AIRFRAMES_DIR/4005_gz_x500_vision" "$AIRFRAMES_DIR/4020_gz_x500_vision_lidar"
sed -i '8s/x500_vision/x500_vision_lidar/' "$AIRFRAMES_DIR/4020_gz_x500_vision_lidar"
sed -i '/# \[22000, 22999\] Reserve for custom models/a\\t4020_gz_x500_vision_lidar' \
  "$AIRFRAMES_DIR/CMakeLists.txt"

# Rebuild PX4 with the new airframe registered (build only, does not launch)
cd ~/px4_workspace/PX4-Autopilot
make px4_sitl_default

# =============================================================================
echo ""
echo "=========================================="
echo " Setup complete!"
echo "=========================================="
echo ""
echo " Paths:"
echo "   PX4:         ~/px4_workspace/PX4-Autopilot"
echo "   Agent:       ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
echo "   ROS2 ws:     ~/ros_ws/drone"
echo ""
echo " To launch (headless):"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/ros_ws/drone/install/setup.bash"
echo "   PX4_DIR=~/px4_workspace/PX4-Autopilot \\"
echo "   XRCE_AGENT=~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent \\"
echo "   ros2 launch drone_bringup mapping_hover.launch.py headless:=true"
echo ""
