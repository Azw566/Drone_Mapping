#!/usr/bin/env bash
# =============================================================================
#  test_arm_takeoff.sh
#
#  Minimal arm + takeoff smoke test for the two-drone simulation.
#  No SLAM, no exploration — just Gazebo + PX4 SITL + offboard arm/takeoff.
#
#  Usage:
#    bash test_arm_takeoff.sh           # with RViz
#    bash test_arm_takeoff.sh --no-rviz # headless
#
#  What this script does:
#    1. Sources ROS2 humble + drone workspace
#    2. Kills any stale Gazebo / PX4 / XRCE processes from a previous run
#    3. Opens monitoring windows in gnome-terminal (after 32 s)
#    4. Runs arm_takeoff_test.launch.py in THIS terminal (Ctrl+C to stop)
#
#  Expected timeline:
#    t=+0s   Gazebo world (maze.sdf)
#    t=+5s   Drone d1 spawns at (-1, -8, 0.5)
#    t=+7s   Drone d2 spawns at (+1, -8, 0.5)
#    t=+8s   ros_gz_bridge + robot_state_publishers
#    t=+10s  PX4 SITL d1 (instance 0, port 8888) + XRCE agent
#            PX4 SITL d2 (instance 1, port 8889) + XRCE agent  (staggered +5s)
#            GCS heartbeat → localhost:18570 / 18571
#    t=+15s  Arming params injected into PX4 via stdin pipe:
#              COM_ARM_WO_GPS=1  COM_RC_IN_MODE=4  NAV_DLL_ACT=0
#              CBRK_SUPPLY_CHK   CBRK_FLIGHTTERM   COM_ARM_CHK_ESCS=0 ...
#    t=+20s  EKF2 initialises from SITL GPS → xy_valid / z_valid = True
#    t=+30s  Offboard controllers start (both drones)
#    t=+32s  d1 & d2: IDLE → PRE_ARM (publish OCM + hold setpoint for 2 s)
#    t=+34s  → SWITCHING  (request OFFBOARD mode; retry every 1 s)
#    t=+35s  → ARMING     (OFFBOARD confirmed; send ARM command; retry every 2 s)
#    t=+36s  → TAKING_OFF (climb to hover_alt = 3.0 m NED z = -3.0)
#    t=+40s  → HOVER      (both drones holding 3 m, waiting for goal_pose)
#
#  Pass criteria (checked in the monitoring windows):
#    nav_state  == 14  (NAVIGATION_STATE_OFFBOARD)
#    arming_state == 2 (ARMED)
#    vehicle_local_position.z ≈ -3.0 m  (hover altitude in NED)
# =============================================================================

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

# ── Colour helpers ─────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[TEST]${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET}   $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
fail()    { echo -e "${RED}[FAIL]${RESET} $*"; }

# ── Args ──────────────────────────────────────────────────────────────────────
USE_RVIZ="true"
for arg in "$@"; do
    [[ "$arg" == "--no-rviz" ]] && USE_RVIZ="false"
done

# ── Pre-flight checks ─────────────────────────────────────────────────────────
if [[ ! -f "$ROS_SETUP" ]]; then
    fail "ROS2 Humble not found at $ROS_SETUP"
    exit 1
fi
if [[ ! -f "$WS_SETUP" ]]; then
    fail "Workspace not built — run: colcon build --symlink-install"
    exit 1
fi

# Source environment
source "$ROS_SETUP"
source "$WS_SETUP"

# ── Kill leftover processes from a previous run ────────────────────────────────
info "Cleaning up stale processes..."
pkill -f "gz sim"             2>/dev/null && warn "Killed stale gz sim"           || true
pkill -f "px4"                2>/dev/null && warn "Killed stale PX4"              || true
pkill -f "MicroXRCEAgent"     2>/dev/null && warn "Killed stale XRCE agent"       || true
pkill -f "offboard_controller" 2>/dev/null && warn "Killed stale offboard_controller" || true
pkill -f "gcs_heartbeat"      2>/dev/null && warn "Killed stale gcs_heartbeat"    || true
sleep 1

# ── Print test overview ────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}   ARM + TAKEOFF SMOKE TEST${RESET}"
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo ""
echo -e "  Workspace : ${WS_DIR}"
echo -e "  RViz      : ${USE_RVIZ}"
echo ""
echo -e "${BOLD}  Timeline:${RESET}"
echo -e "  ${CYAN}t+0s${RESET}   Gazebo"
echo -e "  ${CYAN}t+5s${RESET}   Drone d1 spawns"
echo -e "  ${CYAN}t+7s${RESET}   Drone d2 spawns"
echo -e "  ${CYAN}t+10s${RESET}  PX4 SITL x2 + XRCE agents + GCS heartbeat"
echo -e "  ${CYAN}t+15s${RESET}  Arming params injected (GPS EKF2 mode)"
echo -e "  ${CYAN}t+30s${RESET}  Offboard controllers start"
echo -e "  ${CYAN}t+34s${RESET}  PRE_ARM → SWITCHING → ARMING"
echo -e "  ${CYAN}t+36s${RESET}  TAKING_OFF → HOVER @ 3 m"
echo ""
echo -e "${BOLD}  Pass criteria:${RESET}"
echo -e "  • nav_state  == 14  (OFFBOARD confirmed)"
echo -e "  • arming_state == 2 (ARMED)"
echo -e "  • z position ≈ -3.0 m (NED hover altitude)"
echo ""
echo -e "${BOLD}══════════════════════════════════════════════════════${RESET}"
echo ""
echo -e "  ${YELLOW}Monitoring windows will open automatically at t≈32s${RESET}"
echo -e "  ${YELLOW}Press Ctrl+C in this terminal to stop the test${RESET}"
echo ""

# ── Schedule monitoring windows (open after controllers start) ────────────────
# Each gnome-terminal window runs 'ros2 topic echo' with a clear label.
# We schedule them with 'sleep 32 && gnome-terminal …' in the background
# so they open right when the offboard controllers are entering PRE_ARM.

_MONITOR_DELAY=32

(
    sleep $_MONITOR_DELAY

    # Window 1 — d1 VehicleStatus (nav_state + arming_state)
    # nav_state=14 → OFFBOARD | arming_state=2 → ARMED
    gnome-terminal --title="[d1] vehicle_status" -- bash -c "
        source '${ROS_SETUP}';
        source '${WS_SETUP}';
        echo '========================================';
        echo '  d1 VehicleStatus monitor';
        echo '  nav_state=14  → OFFBOARD confirmed';
        echo '  arming_state=2 → ARMED';
        echo '========================================';
        ros2 topic echo /d1/fmu/out/vehicle_status_v1 \
            | grep -E 'nav_state|arming_state';
        exec bash" 2>/dev/null

    # Window 2 — d2 VehicleStatus
    gnome-terminal --title="[d2] vehicle_status" -- bash -c "
        source '${ROS_SETUP}';
        source '${WS_SETUP}';
        echo '========================================';
        echo '  d2 VehicleStatus monitor';
        echo '  nav_state=14  → OFFBOARD confirmed';
        echo '  arming_state=2 → ARMED';
        echo '========================================';
        ros2 topic echo /d2/fmu/out/vehicle_status_v1 \
            | grep -E 'nav_state|arming_state';
        exec bash" 2>/dev/null

    # Window 3 — d1 altitude (NED z, hover target = -3.0 m)
    gnome-terminal --title="[d1] altitude (z NED)" -- bash -c "
        source '${ROS_SETUP}';
        source '${WS_SETUP}';
        echo '========================================';
        echo '  d1 altitude monitor (NED z)';
        echo '  Target: z ≈ -3.0 m  (xy_valid + z_valid must be true)';
        echo '========================================';
        ros2 topic echo /d1/fmu/out/vehicle_local_position \
            | grep -E '^z:|xy_valid|z_valid';
        exec bash" 2>/dev/null

    # Window 4 — d2 altitude
    gnome-terminal --title="[d2] altitude (z NED)" -- bash -c "
        source '${ROS_SETUP}';
        source '${WS_SETUP}';
        echo '========================================';
        echo '  d2 altitude monitor (NED z)';
        echo '  Target: z ≈ -3.0 m  (xy_valid + z_valid must be true)';
        echo '========================================';
        ros2 topic echo /d2/fmu/out/vehicle_local_position \
            | grep -E '^z:|xy_valid|z_valid';
        exec bash" 2>/dev/null

) &
MONITOR_PID=$!

# ── Launch the test ────────────────────────────────────────────────────────────
info "Launching arm_takeoff_test.launch.py ..."
echo ""

# Trap Ctrl+C to also kill the background monitor scheduler
cleanup() {
    echo ""
    info "Stopping test..."
    kill $MONITOR_PID 2>/dev/null || true
    # Close any monitoring windows (best-effort)
    pkill -f "vehicle_status_v1" 2>/dev/null || true
    pkill -f "vehicle_local_position" 2>/dev/null || true
}
trap cleanup INT TERM

ros2 launch drone_bringup arm_takeoff_test.launch.py "use_rviz:=${USE_RVIZ}"
