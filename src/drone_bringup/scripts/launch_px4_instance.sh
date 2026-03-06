#!/bin/bash
# launch_px4_instance.sh  <instance_id>  <model_name>  <namespace>
#
# Starts one PX4 SITL instance in Gazebo-standalone mode (connects to an
# already-running Gazebo session instead of launching a new one).
#
# After PX4 finishes its boot sequence we inject custom parameters via
# PX4's interactive shell (stdin pipe).
#
# Arguments
#   instance_id  0|1  — uXRCE-DDS listens on port (8888 + instance_id)
#   model_name   name of the Gazebo model to attach to  (e.g. x500_d1)
#   namespace    ROS2 topic namespace for PX4             (e.g. d1)
#
# Environment (can be overridden before calling this script)
#   PX4_DIR   path to PX4-Autopilot source tree
#             default: /home/telemaque/px4_workspace/PX4-Autopilot

set -euo pipefail

INSTANCE="${1:?Usage: $0 <instance_id> <model_name> <namespace>}"
MODEL_NAME="${2:?}"
NS="${3:?}"

PX4_DIR="${PX4_DIR:-/home/telemaque/px4_workspace/PX4-Autopilot}"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${BUILD_DIR}/bin/px4"
STARTUP="${BUILD_DIR}/etc/init.d-posix/rcS"

# ── Pre-flight checks ────────────────────────────────────────────────────────
if [[ ! -x "${PX4_BIN}" ]]; then
    echo "[PX4-${NS}] ERROR: PX4 binary not found: ${PX4_BIN}" >&2
    exit 1
fi
if [[ ! -f "${STARTUP}" ]]; then
    echo "[PX4-${NS}] ERROR: Startup script not found: ${STARTUP}" >&2
    exit 1
fi

# ── Environment for this PX4 instance ───────────────────────────────────────
export PX4_GZ_STANDALONE=1          # attach to running Gazebo — don't start one
export PX4_GZ_MODEL_NAME="${MODEL_NAME}"   # Gazebo model to attach to
export PX4_UXRCE_DDS_NS="${NS}"     # ROS2/DDS topic namespace  (d1 or d2)
export PX4_SYS_AUTOSTART=4001       # x500 multicopter airframe
# Note: UXRCE_DDS_PRT is a PX4 *parameter* (not env var). Both instances default
# to port 8888 and are distinguished by their UXRCE_DDS_KEY (auto-set per instance).

# Clean stale SITL rootfs so stale EEPROM params from a previous run cannot
# override our injected params (e.g. NAV_DLL_ACT != 0 from a prior session).
SITL_ROOTFS="/tmp/px4_sitl_${INSTANCE}"
if [[ -d "${SITL_ROOTFS}" ]]; then
    echo "[PX4-${NS}] Removing stale rootfs: ${SITL_ROOTFS}"
    rm -rf "${SITL_ROOTFS}"
fi

cd "${PX4_DIR}"

echo "[PX4-${NS}] instance=${INSTANCE} model=${MODEL_NAME} rootfs=${BUILD_DIR}"

# ── Inject custom parameters via stdin pipe ──────────────────────────────────
# The co-process below feeds PX4's interactive shell.
# We wait 5 s for PX4 to finish its boot sequence before sending commands.
# "sleep infinity" keeps the pipe open so PX4 doesn't receive EOF.
{
    sleep 5    # wait for PX4 to fully boot before injecting params (SITL boots in ~3 s)

    # Disable all arming checks that don't apply to headless SITL
    echo "param set COM_ARM_WO_GPS 1"        # allow arming without GPS fix
    echo "param set COM_RC_IN_MODE 4"        # no RC required
    echo "param set CBRK_SUPPLY_CHK 894281"  # disable power supply check (no power module in SITL)
    echo "param set NAV_DLL_ACT 0"           # GCS connection not required for arming
    echo "param set COM_DL_LOSS_T 10"        # tolerate 10 s GCS loss before failsafe in flight
    echo "param set CBRK_FLIGHTTERM 121212"  # disable flight termination circuit breaker
    echo "param set COM_ARM_CHK_ESCS 0"      # skip ESC arming check (SITL: no real ESCs)
    echo "param set CBRK_USB_CHK 197848"     # disable USB connected check
    echo "param set COM_CPU_MAX -1"          # disable CPU load check
    echo "param set COM_OF_LOSS_T 10"        # tolerate 10 s OFFBOARD loss before failsafe
                                             # (XRCE-DDS timesync can take ~5 s to reconverge)
    echo "param set UXRCE_DDS_SYNCT 1000"   # increase DDS time-sync tolerance (ms) for high-load SITL

    # Magnetometer: tell PX4 this vehicle has no magnetometer hardware.
    # In SITL the simulated mag cross-check fails when EKF2 propagates IMU-only
    # during XRCE-DDS timesync gaps, cascading into an attitude-failure failsafe.
    # SYS_HAS_MAG 0 disables all compass checks (preflight and in-flight).
    # Yaw is then initialised from GPS velocity, which is stable in SITL.
    echo "param set SYS_HAS_MAG 0"          # no magnetometer — GPS-velocity yaw only
    echo "param set COM_ARM_MAG_ANG -1"     # disable pre-arm magnetometer heading check

    # Adjust hover throttle for our heavier drone.
    # x500_base = 2.0 kg; we add lidar (0.83 kg) + camera (0.05 kg) = 2.88 kg total.
    # Required MPC_THR_HOVER ≈ 0.60 × sqrt(2.88/2.0) ≈ 0.72
    echo "param set MPC_THR_HOVER 0.72"      # hover throttle for 2.88 kg variant

    # Battery failsafe — SITL battery simulation drains quickly and triggers failsafe
    # before the drone has a chance to take off.  Suppress it for SITL testing.
    echo "param set COM_LOW_BAT_ACT 0"       # 0=warning only (no land/RTL on low battery)
    echo "param set BAT_CRIT_THR 0.05"       # critical threshold 5% (default 7%)
    echo "param set BAT_EMERGEN_THR 0.01"    # emergency threshold 1% (default 5%)

    # EKF2: initialise from GPS only. VIO fusion (EKF2_EV_CTRL=15) is enabled
    # in Phase 2 below, after LIO-SAM has had time to produce stable odometry.

    echo "param save"                        # persist params to SITL EEPROM

    echo "[PX4-${NS}] custom parameters applied." >&2

    # ── Phase 2: enable VIO fusion once visual_odom_bridge is publishing ────
    # Skipped when PX4_ENABLE_VIO=0 (e.g. flight_test — GPS only, no LIO-SAM).
    # Enabling EKF2_EV_CTRL=15 before VIO data flows causes EKF2 to diverge
    # and triggers immediate attitude-failure failsafes — confirmed by test.
    #
    # Instead of a blind sleep, we poll the VIO input topic until a message
    # arrives, then wait 3 s for EKF2 to warm up on a few messages.
    # Falls back after 120 s in case something goes wrong (operator can see
    # the warning and diagnose).
    #
    # EKF2_HGT_REF=0 keeps GPS as the primary height source so the drone
    # retains a valid height estimate if LIO-SAM lags behind.
    # EKF2_EV_DELAY=200 accounts for LIO-SAM processing latency (~100–200 ms).
    if [[ "${PX4_ENABLE_VIO:-1}" != "0" ]]; then
        echo "[PX4-${NS}] Waiting for VIO data on /${NS}/fmu/in/vehicle_visual_odometry ..." >&2
        VIO_DETECTED=0
        for _i in $(seq 1 24); do
            if timeout 5 bash -c "
                source /opt/ros/humble/setup.bash 2>/dev/null
                source /home/telemaque/ros_ws/drone/install/setup.bash 2>/dev/null
                ros2 topic echo --qos-reliability best_effort --once \
                    /${NS}/fmu/in/vehicle_visual_odometry > /dev/null 2>&1
            "; then
                VIO_DETECTED=1
                break
            fi
            echo "[PX4-${NS}] VIO not ready (attempt ${_i}/24), retrying in 5 s ..." >&2
            sleep 5
        done

        if [[ ${VIO_DETECTED} -eq 1 ]]; then
            echo "[PX4-${NS}] VIO data confirmed — waiting 3 s before enabling EKF2 fusion ..." >&2
        else
            echo "[PX4-${NS}] WARNING: VIO not detected after 120 s, enabling anyway." >&2
        fi
        sleep 3

        echo "param set EKF2_EV_CTRL 15"        # fuse VIO: horiz pos + vert pos + vel + yaw
        echo "param set EKF2_HGT_REF 0"         # keep GPS as primary height reference
        echo "param set EKF2_EV_DELAY 200"      # LIO-SAM processing latency (~100–200 ms)
        echo "param save"

        echo "[PX4-${NS}] VIO fusion enabled (EKF2_EV_CTRL=15)." >&2
    else
        echo "[PX4-${NS}] VIO fusion skipped (PX4_ENABLE_VIO=0, GPS-only mode)." >&2
    fi

    # Keep the pipe alive — DO NOT send EOF
    sleep infinity

} | "${PX4_BIN}" -i "${INSTANCE}" -d "${BUILD_DIR}/etc"
