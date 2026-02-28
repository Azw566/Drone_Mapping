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
# We wait 20 s for PX4 to finish its boot sequence before sending commands.
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
    echo "param set UXRCE_DDS_SYNCT 500"    # increase DDS time-sync tolerance (ms) for high-load SITL

    # Adjust hover throttle for our heavier drone.
    # x500_base = 2.0 kg; we add lidar (0.83 kg) + camera (0.05 kg) = 2.88 kg total.
    # Required MPC_THR_HOVER ≈ 0.60 × sqrt(2.88/2.0) ≈ 0.72
    echo "param set MPC_THR_HOVER 0.72"      # hover throttle for 2.88 kg variant

    # Battery failsafe — SITL battery simulation drains quickly and triggers failsafe
    # before the drone has a chance to take off.  Suppress it for SITL testing.
    echo "param set COM_LOW_BAT_ACT 0"       # 0=warning only (no land/RTL on low battery)
    echo "param set BAT_CRIT_THR 0.05"       # critical threshold 5% (default 7%)
    echo "param set BAT_EMERGEN_THR 0.01"    # emergency threshold 1% (default 5%)

    # EKF2: use GPS for arming (LIO-SAM VIO integration is a separate concern).
    # EKF2_EV_CTRL left at default (0) so EKF2 initialises from GPS immediately.
    # Note: EKF2_EV_CTRL=15 to blend VIO cannot be set here because LIO-SAM
    # needs ~30 s to initialise; setting it before VIO flows causes ekf2 missing data.

    echo "param save"                        # persist params to SITL EEPROM

    echo "[PX4-${NS}] custom parameters applied." >&2

    # Keep the pipe alive — DO NOT send EOF
    sleep infinity

} | "${PX4_BIN}" -i "${INSTANCE}" -d "${BUILD_DIR}/etc"
