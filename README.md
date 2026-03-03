# Multi-Drone Autonomous Exploration

ROS 2 (Humble) + PX4 v1.16 + Gazebo Harmonic stack for coordinated autonomous exploration with two `x500_vision_lidar` drones in a simulated maze environment.

Each drone runs LiDAR-inertial SLAM (LIO-SAM), 3-D occupancy mapping (OctoMap), frontier-based exploration, and ArUco marker detection, all coordinated through a central exploration manager.

---

## Architecture

```
Gazebo sensors
  ├─ /d{n}/points_raw   → lidar_enricher → /d{n}/points_enriched
  │                                              │
  │                                          LIO-SAM ──► /d{n}/lio_sam/mapping/odometry
  │                                                  └──► /d{n}/lio_sam/mapping/cloud_registered
  │                                                              │
  │                                                       octomap_server ──► /d{n}/projected_map
  │                                                                                   │
  │                                                                         frontier_detector ──► /d{n}/frontiers/list
  │                                                                                                       │
│                                                                                         drone_coordinator ──► assign_frontier
  │                                                                                                                       │ │                                                                                                         exploration_planner ──► /d{n}/goal_pose │                                                                                                                                         │ │                                                                                                                            offboard_controller ──► PX4 setpoints
  │
  └─ /d{n}/camera/image_raw ──► aruco_detector ──► /d{n}/aruco/detections ──► poi_manager

PX4 SITL ◄──► MicroXRCE-DDS Agent ◄──► visual_odom_bridge (LIO-SAM ENU → PX4 NED)
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

## Drone model

**`x500_vision_lidar`** extends the official PX4 `x500` airframe with:

| Sensor | Spec | Mount |
|--------|------|-------|
| VLP-16 LiDAR | 16 rings × 1800 pts, 10 Hz | Top (+0.12 m) |
| RGB Camera | 640 × 480, 15 FPS | Front, 15° down pitch |

TF tree: `{ns}/map → {ns}/odom → {ns}/base_link → {ns}/camera_link → {ns}/camera_optical_frame`

> The `x500_vision_lidar` model must be installed in PX4's model directory.
> See [`src/drone_bringup/models/x500_vision_lidar/SETUP.md`](src/drone_bringup/models/x500_vision_lidar/SETUP.md).

---

## Prerequisites

| Dependency | Version | Path |
|------------|---------|------|
| Ubuntu | 22.04 | — |
| ROS 2 | Humble | `/opt/ros/humble` |
| Gazebo | Harmonic (gz-sim 8) | system install |
| PX4-Autopilot | v1.16.0 | `~/px4_workspace/PX4-Autopilot` |
| Micro-XRCE-DDS-Agent | latest | `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent` |
| Python | 3.10+ | — |
| OpenCV | 4.x with ArUco contrib | — |
| GTSAM, PCL, `ros-humble-perception-pcl` | — | LIO-SAM dependencies |

---

## Installation

```bash
# Install ROS 2 dependencies
cd ~/ros_ws/drone
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> If `colcon build` fails with *"existing path cannot be removed: Is a directory"*, remove the conflicting directory and rebuild:
> ```bash
> rm -rf build/<pkg>/ament_cmake_python/<pkg>/<pkg>
> colcon build --symlink-install
> ```

---

## Before every launch — kill stale processes

Leftover processes from a previous run corrupt PX4 SITL EEPROM params and block port binding.

```bash
pkill -f "gz sim" 2>/dev/null
pkill -f "px4" 2>/dev/null
pkill -f "MicroXRCEAgent" 2>/dev/null
pkill -f "offboard_controller\|exploration_planner\|frontier_detector\|drone_coordinator\|octomap_server\|aruco_detector\|visual_odom_bridge" 2>/dev/null
sleep 3
rm -rf /tmp/px4_sitl_0 /tmp/px4_sitl_1
```

---

## Quick start

### Full stack (recommended)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup full_stack.launch.py
```

**With RViz:**
```bash
ros2 launch drone_bringup full_stack.launch.py use_rviz:=true
```

**Headless (no Gazebo window — saves ~40 % CPU):**
```bash
ros2 launch drone_bringup full_stack.launch.py headless:=true
```

**Open Gazebo GUI after a headless launch** (connect to running server):
```bash
gz sim   # in a second terminal, no relaunch needed
```

### Staged launch sequence

`full_stack.launch.py` brings up the pipeline in order:

| Time | Components started |
|------|--------------------|
| t = 0 s | Gazebo world loads |
| t = 5 s | d1 drone spawned |
| t = 7 s | d2 drone spawned |
| t = 8 s | `ros_gz_bridge` + robot state publishers |
| t = 10 s | PX4 SITL × 2 + MicroXRCE-DDS Agent × 2 (ports 8888 / 8889) |
| t = 12 s | LIO-SAM × 2 + `visual_odom_bridge` × 2 + GCS heartbeat |
| t = 15 s | OctoMap × 2, ArUco detectors × 2 |
| t = 15 s | d1 custom PX4 params injected via stdin pipe |
| t = 17 s | Frontier detectors × 2 |
| t = 20 s | d2 custom PX4 params injected |
| t = 30 s | Offboard controllers × 2 |
| t = 33 s | Exploration planners × 2 + drone coordinator + POI manager |
| t ≈ 50 s | EKF2 VIO fusion enabled on d1 (Phase 2 param injection) |
| t ≈ 55 s | EKF2 VIO fusion enabled on d2 |

### Arm + takeoff test only

Minimal test — no SLAM, no exploration. Verifies both drones arm, switch to OFFBOARD, and hover at 3 m using PX4 built-in SITL GPS.

```bash
ros2 launch drone_bringup arm_takeoff_test.launch.py
```

### Manual multi-terminal launch

```bash
# Terminal 1 — Gazebo + bridge
ros2 launch drone_bringup simulation.launch.py

# Terminal 2 — PX4 SITL × 2 + XRCE agents
ros2 launch drone_bringup px4_multi.launch.py

# Terminal 3 — LIO-SAM × 2
ros2 launch drone_bringup lio_sam_multi.launch.py

# Terminal 4 — Offboard controllers
ros2 launch px4_offboard px4_offboard.launch.py

# Terminal 5 — Exploration stack
ros2 launch exploration_manager exploration_manager.launch.py
```

---

## Monitoring a running stack

Open a second terminal:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

### Sensor pipeline
```bash
ros2 topic hz /d1/points_raw                        # ~10 Hz  — LiDAR from bridge
ros2 topic hz /d1/imu/data                          # ~100 Hz — IMU from bridge
ros2 topic hz /d1/camera/image_raw                  # ~15 Hz  — camera from bridge
ros2 topic hz /d1/lio_sam/mapping/cloud_registered  # ~1 Hz   — LIO-SAM map cloud
ros2 topic hz /d1/lio_sam/mapping/odometry          # ~10 Hz  — LIO-SAM odometry
ros2 topic hz /d1/projected_map                     # ~1–2 Hz — OctoMap grid
ros2 topic hz /d1/frontiers/list                    # ~2 Hz   — frontier clusters
```

### Drone state and goals
```bash
ros2 topic echo /d1/drone_state --once              # status, current_goal, battery
ros2 topic echo /d1/goal_pose --once                # exploration waypoint (ENU)
ros2 topic echo /mission_complete                   # Bool: True when done
```

### PX4 internals
```bash
ros2 topic echo /d1/fmu/out/vehicle_status_v1 --once   # nav_state=14 → OFFBOARD, arming_state=2 → ARMED
ros2 topic echo /d1/fmu/out/vehicle_local_position --once
```

---

## Expected console output

| Time | Log message |
|------|-------------|
| ~t = 0 s | `[gz-4] [Msg] Loaded world [maze]` |
| ~t = 10 s | `[px4_d1] INFO [init] PX4 v1.16.0` |
| ~t = 15 s | `[PX4-d1] custom parameters applied` |
| ~t = 20 s | `[PX4-d2] custom parameters applied` |
| ~t = 30–35 s | `[d1] Home NED: [x, y, z]` — EKF2 has position fix |
| ~t = 30–35 s | `[d1] → PRE_ARM → SWITCHING → ARMING → TAKING_OFF → HOVER` |
| ~t = 50 s | `[PX4-d1] VIO fusion enabled (EKF2_EV_CTRL=15)` |
| ~t = 55 s | `[PX4-d2] VIO fusion enabled (EKF2_EV_CTRL=15)` |
| ~t = 40 s | `[frontier_detector] Frontiers: N cluster(s) detected` |
| ~t = 40 s | `[coordinator] d1 accepted frontier (x.x, y.y)` |
| later | `[coordinator] Mission complete — all frontiers exhausted` |
| later | `[d1] Land command → LANDING` → `[d1] Landed → IDLE` |

---

## Test checklist

### Level 1 — Simulation starts
- [ ] Gazebo window opens (or `gz topic -l` returns maze topics)
- [ ] No `[ERROR]` from `gz_bridge` in the first 10 s
- [ ] Both drones visible at spawn position

### Level 2 — PX4 and arming
- [ ] `[PX4-d1] custom parameters applied` at ~t = 15 s
- [ ] `[PX4-d2] custom parameters applied` at ~t = 20 s
- [ ] `[d1] Home NED:` printed (EKF2 position valid)
- [ ] `[d2] Home NED:` printed
- [ ] Both drones reach `→ HOVER` at ~1.5 m altitude
- [ ] `[PX4-d1] VIO fusion enabled (EKF2_EV_CTRL=15)` at ~t = 50 s
- [ ] `[PX4-d2] VIO fusion enabled (EKF2_EV_CTRL=15)` at ~t = 55 s
- [ ] No `[failsafe] emergency landing` or `[failsafe] RTL` (battery tone is normal)

### Level 3 — Sensor data and SLAM
- [ ] `/d1/points_raw` at ~10 Hz
- [ ] `/d1/lio_sam/mapping/cloud_registered` at ~1 Hz ← key: confirms lidar field alignment OK
- [ ] `/d1/lio_sam/mapping/odometry` publishing
- [ ] `/d1/projected_map` publishing
- [ ] RViz shows OccupancyGrid growing as drones move

### Level 4 — Exploration
- [ ] `Frontiers: N cluster(s) detected` appears
- [ ] `accepted frontier (x.x, y.y)` appears for both drones
- [ ] Drones visibly move toward frontier targets
- [ ] Frontier count decreases over time
- [ ] ArUco log appears if a marker is visible: `ArUco tag #N detected`

### Level 5 — Mission complete
- [ ] `Mission complete — all frontiers exhausted` logged
- [ ] Both drones receive land command and reach `→ IDLE`
- [ ] `/mission_complete` publishes `data: True`

### Level 6 — Map capture

> Run this in a **second terminal while the stack is still running** — the map is lost when the launch is killed.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/show_map_result.py --output /tmp/maze_result.png
```

- [ ] Script prints `[OK] Map saved → /tmp/maze_result.png`
- [ ] PNG shows white free space, black walls, red circles for any ArUco POIs
- [ ] Text report shows non-zero explored area (m²) for both drones

---

## Troubleshooting

### Drones not taking off — stuck in IDLE or PRE_ARM

**Symptom:** `[d1] Home NED` never prints.
**Cause:** EKF2 has no position fix yet (GPS cold start or VIO not flowing).
**Fix:** Wait up to 45 s. If still stuck, check `[ekf2]` lines in the PX4 output.

---

### Drones hover but never explore

**Symptom:** Both drones hover at 1.5 m, no `accepted frontier` log.
**Diagnose:** Follow the chain from the bottom up:
```bash
ros2 topic hz /d1/lio_sam/mapping/cloud_registered  # 0 Hz → LIO-SAM broken
ros2 topic hz /d1/projected_map                     # 0 Hz → OctoMap not getting cloud
ros2 topic hz /d1/frontiers/list                    # 0 Hz → no map → no frontiers
```

---

### `cloud_registered` at 0 Hz despite LIO-SAM running

**Symptom:** `/d1/lio_sam/mapping/odometry` publishes but `cloud_registered` is silent.
**Cause:** `ring` / `time` PointCloud2 field misalignment — LIO-SAM discards every scan.
**Verify the fix is present:**
```bash
grep "offset % 4" src/drone_bringup/scripts/lidar_enricher.py
# Must print:  if offset % 4 != 0:
```
If the line is missing, rebuild:
```bash
colcon build --symlink-install --packages-select drone_bringup
```

---

### EKF2 VIO log never appears

**Symptom:** `[PX4-d1] VIO fusion enabled` never prints.
**Cause:** Phase 2 stdin pipe to PX4 failed.
**Diagnose:** Check that `[PX4-d1] custom parameters applied` (Phase 1) appeared. If even Phase 1 is missing, the PX4 stdin pipe is broken — check `launch_px4_instance.sh`.
**Note:** GPS is still active; the drone will fly without VIO fusion, just with less accurate localisation.

---

### QoS incompatibility warning on `projected_map`

**Symptom:** `[octomap_server] DURABILITY_QOS_POLICY incompatible`
**Fix:** Confirm `latch: true` is in `octomap_params.yaml`:
```bash
grep latch src/octomap_pipeline/params/octomap_params.yaml
# Must print:  latch: true
```

---

### Gazebo window freezes / all topics go to 0 Hz

**Cause:** Gazebo server crashed. Python ROS 2 nodes continue running but receive nothing.
**Fix:** Run the kill command at the top of this guide and relaunch.

---

### PX4 time sync warnings

```
WARN [timesync] time jump detected
WARN [uxrce_dds_client] time sync no longer converged
INFO [uxrce_dds_client] time sync converged
```
**Status:** Normal — caused by Gazebo clock jitter under load. Self-corrects within seconds.

---

### Battery failsafe tone at startup

```
WARN [failsafe] Failsafe activated
INFO [tone_alarm] battery warning (fast)
```
**Status:** Normal. `COM_LOW_BAT_ACT=0` is injected at t = 15 s so no RTL or landing is triggered. Tone only.

---

### `flight_mode_manager` error at startup

```
ERROR [flight_mode_manager] Matching flight task was not able to run, Nav state: 2, Task: 1
```
**Status:** Normal transient — EKF2 is not initialised when PX4 first tries POSCTL. Clears within 10 s.

---

## Configuration

| File | Purpose |
|------|---------|
| `setup_env.bash` | Shell environment (`GZ_HOMEDIR`, `ROS_LOG_DIR`, `GZ_SIM_RESOURCE_PATH`) |
| `drone_bringup/config/lio_sam_d1.yaml` | LIO-SAM params for d1 (`pointCloudTopic: /d1/points_enriched`) |
| `drone_bringup/config/bridge.yaml` | `ros_gz_bridge` topic mappings |
| `octomap_pipeline/params/octomap_params.yaml` | OctoMap server params |

---

## Log locations

| Log type | Location |
|----------|----------|
| PX4 ULog | `~/px4_workspace/PX4-Autopilot/build/px4_sitl_default/log/YYYY-MM-DD/*.ulg` |
| ROS 2 logs | `.ros/log/` (or `$ROS_LOG_DIR` if `setup_env.bash` was sourced) |
| Gazebo state | `.gz/` |

---

## Project structure

```
drone/
├── setup_env.bash                  # Environment setup (Gazebo paths, log dirs)
├── scripts/
│   ├── show_map_result.py          # Capture final map + POIs as PNG after mission
│   ├── test_exploration_cycle.py   # No-sim integration test (planner + coordinator)
│   └── test_soak.py                # Sustained-load soak test
├── src/
│   ├── drone_bringup/              # Launch files, Gazebo worlds, drone models, bridge config
│   │   ├── config/                 # LIO-SAM params, bridge topic mappings
│   │   ├── launch/                 # full_stack, simulation, px4_multi, lio_sam_multi, …
│   │   ├── models/                 # x500_vision_lidar SDF + PX4 airframe
│   │   ├── scripts/                # lidar_enricher, launch_px4_instance.sh, gcs_heartbeat
│   │   └── worlds/                 # maze.sdf
│   ├── drone_interfaces/           # Custom ROS 2 messages and services
│   ├── drone_slam/                 # PointCloud / IMU adapter nodes
│   ├── px4_offboard/               # VIO bridge + offboard flight controller
│   ├── px4_msgs/                   # PX4 message definitions
│   ├── LIO-SAM/                    # LiDAR-inertial SLAM (C++)
│   ├── octomap_pipeline/           # OctoMap integration
│   ├── frontier_detector/          # Frontier-based exploration detector
│   ├── aruco_detector/             # ArUco marker detection
│   └── exploration_manager/        # Multi-drone coordination and planning
```
# Mapping_Multiple_Drone
# Drone_Automatic_Mapping
# Drone_Mapping
