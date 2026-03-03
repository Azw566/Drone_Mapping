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
at 1.5 m. Does **not** require LIO-SAM or exploration nodes.

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

