# Test Checklist — Multi-Drone Autonomous Exploration

This document describes how to verify that every layer of the stack is working
correctly, from pure-logic unit tests through to a full Gazebo simulation run.

---

## Prerequisites

```bash
# Build the workspace
source /opt/ros/humble/setup.bash
cd ~/ros_ws/drone
colcon build --symlink-install
source install/setup.bash

# One-time: install PX4 x500_vision_lidar model
# See src/drone_bringup/models/x500_vision_lidar/SETUP.md
```

---

## Level 1 — No-Sim Tests (no Gazebo / PX4 required)

These tests verify the exploration logic layer in isolation.
Run them **in a fresh terminal** with the workspace sourced.

### 1a. Exploration cycle integration test

Verifies the full frontier-assign → navigate → reach-goal → idle → reassign
cycle using mocked odometry and frontiers.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/test_exploration_cycle.py
```

**Pass criteria:**
- `[PASS] Both nodes are running after startup`
- `[PASS] goal_pose near frontier_1 received within 8.0s`
- `[PASS] drone_state reached exploring before idle`
- `[PASS] drone_state transitions to idle within 8.0s after reaching goal`
- `[PASS] second goal_pose near frontier_2 received within 8.0s`
- Final: `[PASS] All assertions passed — exploration cycle works end-to-end.`
- Exit code: `0`

> Note: `ExternalShutdownException` in the output is harmless rclpy Humble
> teardown noise and does **not** indicate a failure.

**Confirmed result (2026-03-01):** PASS — all 5 assertions.

---

### 1b. Sustained-load soak test

Runs the exploration stack under continuous load for a configurable duration.
To avoid DDS residual state from other tests, use a dedicated domain ID.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 python3 scripts/test_soak.py --duration 60
```

For a quick smoke test:
```bash
ROS_DOMAIN_ID=42 python3 scripts/test_soak.py --duration 30
```

**Pass criteria:**
- `Cycles completed: N` where N > 0
- `Errors: 0`
- Final: `[PASS] Soak test passed — N cycles in Xs, no errors.`
- Exit code: `0`

> Important: always pass `ROS_DOMAIN_ID=42` (or any non-default domain) when
> running this test immediately after another ROS 2 test in the same session.
> Without it, residual DDS discovery entries from the previous run can cause
> intermittent timeouts.

**Confirmed result (2026-03-01):** PASS — 60 cycles in 60s, 0 errors.

---

## Level 2 — Arm + Takeoff Test (Gazebo + PX4, no exploration)

Verifies that both drones arm, switch to OFFBOARD mode, take off, and hover
at 3 m. Does **not** require LIO-SAM or exploration nodes.

```bash
source setup_env.bash
ros2 launch drone_bringup arm_takeoff_test.launch.py
```

**Pass criteria (check after ~60 s):**

| Check | Command | Expected |
|-------|---------|----------|
| d1 nav_state | `ros2 topic echo /d1/fmu/out/vehicle_status_v1 --once` | `nav_state: 14` (OFFBOARD) |
| d2 nav_state | `ros2 topic echo /d2/fmu/out/vehicle_status_v1 --once` | `nav_state: 14` |
| d1 arming | same message | `arming_state: 2` (ARMED) |
| d1 altitude | `ros2 topic echo /d1/fmu/out/vehicle_local_position --once` | `z` ≈ `-3.0` (NED) |
| d2 altitude | `ros2 topic echo /d2/fmu/out/vehicle_local_position --once` | `z` ≈ `-3.0` |

Motor RPM (Gazebo): all four motors should show ~900–950 rad/s with balanced
pairs (yaw must be `NaN` in setpoints — see `offboard_controller_node.py`).

**Cleanup:**
```bash
ps aux | grep -E "(gz sim|px4|MicroXRCE|offboard_controller|gcs_heartbeat)" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2
rm -rf /tmp/px4_sitl_0 /tmp/px4_sitl_1
```

**Confirmed result (2026-02-28):** PASS — both drones hover at 3 m. ✓
Root cause of earlier failure (yaw=0.0 → motor saturation) was fixed by
setting `msg.yaw = float('nan')` in all setpoint publishers.

---

## Level 3 — Full-Stack Launch (complete pipeline)

Starts everything in staged order (~33 s total warmup).

```bash
source setup_env.bash
ros2 launch drone_bringup full_stack.launch.py use_rviz:=false
```

Wait ~40 s for all nodes to initialise, then run the topic verification script:

```bash
# In a second terminal (workspace sourced)
python3 scripts/test_rviz_topics.py
```

### Manual topic checks

After the stack is running, verify key data flows:

```bash
# LIO-SAM odometry active (expect ~10 Hz)
ros2 topic hz /d1/lio_sam/mapping/odometry

# OctoMap grid publishing
ros2 topic echo /d1/projected_map --once | head -5

# Frontier detection active
ros2 topic echo /d1/frontiers/list --once

# Offboard / navigation state (expect nav_state=14, arming_state=2)
ros2 topic echo /d1/fmu/out/vehicle_status_v1 --once

# ArUco detections (walk camera past a marker)
ros2 topic echo /d1/aruco/detections

# Mission complete signal (published when all frontiers exhausted)
ros2 topic echo /mission_complete --once
```

**Pass criteria for full stack:**

| Topic | Expected |
|-------|----------|
| `/d1/lio_sam/mapping/odometry` | ~10 Hz |
| `/d1/lio_sam/mapping/cloud_registered` | ~10 Hz |
| `/d1/projected_map` | published (latched) |
| `/d1/frontiers/list` | published |
| `/d1/frontiers/markers` | published |
| `/d1/fmu/out/vehicle_status_v1` | `nav_state=14`, `arming_state=2` |
| `/d1/fmu/out/vehicle_local_position` | `z ≈ -3.0` |
| `/d1/goal_pose` | published when frontiers available |
| `/d1/drone_state` | `status: exploring` |
| `/mission_complete` | `True` after all frontiers exhausted |

**`test_rviz_topics.py` pass criteria:**
```
[PASS] All enabled RViz topics are present with correct types.
```

---

## Level 4 — End-to-End Mission (manual observation)

With the full stack running, observe in RViz (`use_rviz:=true`) or via topic
echoes that:

1. Both drones arm and take off to 3 m (**HOVER** state).
2. Frontier markers appear in the maze as LIO-SAM maps the area.
3. The coordinator assigns frontiers to each drone; drones navigate to them.
4. ArUco markers in the maze are detected and registered via `/register_poi`.
5. When no new frontiers appear for `no_frontier_timeout` seconds (default 30 s),
   `/mission_complete` is published and both drones land.

---

## Cleanup

Between test runs:

```bash
# Kill all simulation processes
ps aux | grep -E "(gz sim|px4|MicroXRCE|offboard_controller|gcs_heartbeat)" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
sleep 2
rm -rf /tmp/px4_sitl_0 /tmp/px4_sitl_1

# For no-sim tests, also kill leftover exploration nodes
pkill -9 -f "exploration_planner_node"
pkill -9 -f "drone_coordinator_node"
```

---

## Known Issues / Notes

| Issue | Status | Notes |
|-------|--------|-------|
| `ExternalShutdownException` in no-sim tests | Harmless | rclpy Humble teardown noise; exit code is still 0 on PASS |
| Soak test intermittent failure with default `ROS_DOMAIN_ID` | Workaround | Use `ROS_DOMAIN_ID=42` to isolate DDS domain |
| `offboard_controller` must subscribe to `vehicle_status_v1` (with `_v1` suffix) | Fixed | PX4 v1.16 SITL publishes with this suffix |
| `msg.yaw = 0.0` causes motor saturation | Fixed | Changed to `float('nan')` to hold current heading |
| Battery failsafe must be disabled for SITL | Fixed | `COM_LOW_BAT_ACT 0`, `BAT_CRIT_THR 0.05`, `BAT_EMERGEN_THR 0.01` in `launch_px4_instance.sh` |
