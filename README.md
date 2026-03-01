# Multi-Drone Autonomous Exploration

ROS 2 (Humble) + PX4 v1.16 + Gazebo Harmonic stack for coordinated autonomous exploration with two `x500_vision_lidar` drones in a simulated maze environment.

Each drone runs LiDAR-inertial SLAM (LIO-SAM), 3-D occupancy mapping (OctoMap), frontier-based exploration, and ArUco marker detection, all coordinated through a central exploration manager.

---

## Architecture

```
Gazebo ──► ros_gz_bridge ──► /d{n}/points_raw
                                  │
                             lidar_enricher
                                  │
                             LIO-SAM ──► /d{n}/lio_sam/mapping/odometry
                                    │ └► /d{n}/lio_sam/mapping/cloud_registered
                                    │
                             octomap_server ──► /d{n}/projected_map
                                              │
                                         frontier_detector ──► /d{n}/frontiers/list

Gazebo ──► /d{n}/camera/image_raw ──► aruco_detector ──► /d{n}/aruco/detections

PX4 SITL ◄──► MicroXRCE-DDS Agent ◄──► visual_odom_bridge / offboard_controller
```

### ROS 2 packages

| Package | Language | Role |
|---------|----------|------|
| `drone_bringup` | CMake | Gazebo world, drone models, `ros_gz_bridge`, all launch files |
| `drone_interfaces` | CMake (rosidl) | Custom msgs (`ArucoDetection`, `FrontierList`, `DroneState`) and srvs |
| `drone_slam` | Python | PointCloud adapter + IMU converter nodes |
| `px4_offboard` | Python | `visual_odom_bridge` (ENU→NED) + `offboard_controller` (arm / takeoff / navigate) |
| `px4_msgs` | CMake | PX4 message definitions (matched to PX4 v1.16.0) |
| `LIO-SAM` | C++ | LiDAR-inertial odometry and mapping |
| `octomap_pipeline` | Python | Wraps `octomap_server`; produces 3-D voxel map + 2-D `projected_map` |
| `frontier_detector` | Python | BFS-clustered frontier detection on `OccupancyGrid`; publishes `FrontierList` + `MarkerArray` |
| `aruco_detector` | Python | OpenCV ArUco detection on RGB camera |
| `exploration_manager` | Python | Per-drone goal tracking, frontier assignment coordinator, ArUco POI deduplication |

---

## Prerequisites

| Dependency | Version |
|------------|---------|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Gazebo | Harmonic |
| PX4-Autopilot | v1.16.0 (at `~/px4_workspace/PX4-Autopilot`) |
| Micro-XRCE-DDS-Agent | latest (at `~/Micro-XRCE-DDS-Agent`) |
| Python | 3.10+ |
| OpenCV | 4.x (with ArUco contrib) |
| LIO-SAM dependencies | GTSAM, PCL, `ros-humble-perception-pcl` |

> The `x500_vision_lidar` drone model must be installed in PX4's model directory.
> See [`src/drone_bringup/models/x500_vision_lidar/SETUP.md`](src/drone_bringup/models/x500_vision_lidar/SETUP.md) for instructions.

---

## Installation

```bash
# Clone into a colcon workspace
mkdir -p ~/ros_ws/drone/src
cd ~/ros_ws/drone

# Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## Quick Start

### One-command launch (recommended)

```bash
source /opt/ros/humble/setup.bash
source setup_env.bash          # sets GZ_HOMEDIR, ROS_LOG_DIR, GZ_SIM_RESOURCE_PATH

ros2 launch drone_bringup full_stack.launch.py use_rviz:=false
```

`full_stack.launch.py` starts everything in staged order (~35 s total):
1. **t = 0 s** — Gazebo + `ros_gz_bridge`
2. **t = 5 s** — PX4 SITL × 2 + MicroXRCE-DDS Agent × 2 (ports 8888 / 8889)
3. **t = 15 s** — LIO-SAM × 2 + `visual_odom_bridge` × 2
4. **t = 22 s** — OctoMap × 2, frontier detector × 2, ArUco detector × 2, exploration manager, `offboard_controller` × 2

### Headless (CI / no display)

```bash
ros2 launch drone_bringup full_stack.launch.py use_rviz:=false
# GZ_HEADLESS=1 is set automatically when use_rviz:=false
```

### With RViz

```bash
ros2 launch drone_bringup full_stack.launch.py use_rviz:=true
```

### Arm + takeoff only (development)

```bash
ros2 launch drone_bringup arm_takeoff_test.launch.py
```

---

## Manual Multi-Terminal Launch

If you need to bring up components individually:

```bash
# Terminal 1 — Gazebo + bridge
source setup_env.bash
ros2 launch drone_bringup simulation.launch.py use_rviz:=false

# Terminal 2 — PX4 instances + XRCE agents
ros2 launch drone_bringup px4_multi.launch.py

# Terminal 3 — LIO-SAM
ros2 launch drone_bringup lio_sam_multi.launch.py

# Terminal 4 — Offboard controllers
ros2 launch px4_offboard px4_offboard.launch.py

# Terminal 5 — Exploration stack
ros2 launch exploration_manager exploration_manager.launch.py
```

---

## Configuration

| File | Purpose |
|------|---------|
| `setup_env.bash` | Shell environment (PX4 path, Gazebo resource paths, log dirs) |
| `drone_bringup/config/lio_sam_d1.yaml` | LIO-SAM params for drone 1 |
| `drone_bringup/config/bridge.yaml` | `ros_gz_bridge` topic mappings |
| `octomap_pipeline/params/octomap_params.yaml` | OctoMap server params |

Key environment variables set by `setup_env.bash`:

| Variable | Default |
|----------|---------|
| `PX4_DIR` | `~/px4_workspace/PX4-Autopilot` |
| `GZ_HOMEDIR` | `<repo>/.gz` |
| `ROS_LOG_DIR` | `<repo>/.ros/log` |
| `GZ_SIM_RESOURCE_PATH` | Appended with repo `models/` and `worlds/` |

---

## Drone Model

**`x500_vision_lidar`** extends the official PX4 `x500` airframe with:

| Sensor | Spec | Mount |
|--------|------|-------|
| VLP-16 LiDAR | 16 rings × 1800 pts, 10 Hz | Top (+0.12 m) |
| RGB Camera | 640 × 480, 15 FPS | Front, 15° down pitch |

TF tree: `{ns}/map → {ns}/odom → {ns}/base_link → {ns}/camera_link → {ns}/camera_optical_frame`

---

## Verifying a Running Stack

See [`TEST_CHECKLIST.md`](TEST_CHECKLIST.md) for the full verification procedure.

Quick topic checks:

```bash
# Offboard state (expect nav_state=14, arming_state=2)
ros2 topic echo /d1/fmu/out/vehicle_status_v1 --once

# LIO-SAM odometry
ros2 topic hz /d1/lio_sam/mapping/odometry

# OctoMap grid
ros2 topic echo /d1/projected_map --once | head -20

# Frontiers
ros2 topic echo /d1/frontiers/list --once

# ArUco detections
ros2 topic echo /d1/aruco/detections
```

---

## Log Locations

| Log type | Location |
|----------|----------|
| PX4 ULog | `~/px4_workspace/PX4-Autopilot/build/px4_sitl_default/log/YYYY-MM-DD/*.ulg` |
| ROS 2 logs | `.ros/log/` |
| Gazebo state | `.gz/` |

---

## Cleanup

Kill all simulation processes and reset PX4 SITL state:

```bash
ps aux | grep -E "(gz sim|px4|MicroXRCE|offboard_controller|gcs_heartbeat)" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2
rm -rf /tmp/px4_sitl_0 /tmp/px4_sitl_1
```

---

## Project Structure

```
drone/
├── setup_env.bash                 # Environment setup script
├── TEST_CHECKLIST.md              # Verification checklist for a full run
├── src/
│   ├── drone_bringup/             # Launch files, Gazebo worlds, drone models, bridge config
│   ├── drone_interfaces/          # Custom ROS 2 messages and services
│   ├── drone_slam/                # PointCloud / IMU adapter nodes
│   ├── px4_offboard/              # VIO bridge + offboard flight controller
│   ├── px4_msgs/                  # PX4 message definitions
│   ├── LIO-SAM/                   # LiDAR-inertial SLAM
│   ├── octomap_pipeline/          # OctoMap integration
│   ├── frontier_detector/         # Frontier-based exploration detector
│   ├── aruco_detector/            # ArUco marker detection
│   └── exploration_manager/       # Multi-drone coordination and planning
└── scripts/                       # Utility scripts
```
