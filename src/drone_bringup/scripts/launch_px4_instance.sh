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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAIT_FOR_VIO="${SCRIPT_DIR}/wait_for_vio.py"

# ── Pre-flight checks ────────────────────────────────────────────────────────
if [[ ! -x "${PX4_BIN}" ]]; then
    echo "[PX4-${NS}] ERROR: PX4 binary not found: ${PX4_BIN}" >&2
    exit 1
fi
if [[ ! -f "${STARTUP}" ]]; then
    echo "[PX4-${NS}] ERROR: Startup script not found: ${STARTUP}" >&2
    exit 1
fi
if [[ ! -x "${WAIT_FOR_VIO}" ]]; then
    echo "[PX4-${NS}] ERROR: wait_for_vio.py not found or not executable: ${WAIT_FOR_VIO}" >&2
    exit 1
fi

# ── Environment for this PX4 instance ───────────────────────────────────────
export PX4_GZ_STANDALONE=1          # attach to running Gazebo — don't start one
export PX4_GZ_MODEL_NAME="${MODEL_NAME}"   # Gazebo model to attach to
export PX4_UXRCE_DDS_NS="${NS}"     # ROS2/DDS topic namespace  (d1 or d2)
export PX4_UXRCE_DDS_PORT="${PX4_UXRCE_DDS_PORT:-$((8888 + INSTANCE))}"
export PX4_SYS_AUTOSTART=4001       # x500 multicopter airframe
# Apply critical PX4 parameters before rcS starts commander / uXRCE-DDS.
# This is the only point where reboot_required params affect the current boot,
# and it also prevents transient preflight/failsafe states during startup.
export PX4_PARAM_UXRCE_DDS_PRT="${PX4_PARAM_UXRCE_DDS_PRT:-${PX4_UXRCE_DDS_PORT}}"
export PX4_PARAM_UXRCE_DDS_PTCFG="${PX4_PARAM_UXRCE_DDS_PTCFG:-1}"
# All ROS→PX4 publishers in this repo stamp messages from PX4's own clock, so
# agent-side timestamp synchronization is redundant and prone to false resets
# under heavy multi-drone DDS load.
export PX4_PARAM_UXRCE_DDS_SYNCT="${PX4_PARAM_UXRCE_DDS_SYNCT:-0}"
export PX4_PARAM_UXRCE_DDS_TX_TO="${PX4_PARAM_UXRCE_DDS_TX_TO:-10}"
export PX4_PARAM_UXRCE_DDS_RX_TO="${PX4_PARAM_UXRCE_DDS_RX_TO:-10}"
export PX4_PARAM_COM_ARM_WO_GPS="${PX4_PARAM_COM_ARM_WO_GPS:-1}"
export PX4_PARAM_COM_RC_IN_MODE="${PX4_PARAM_COM_RC_IN_MODE:-4}"
export PX4_PARAM_COM_DL_LOSS_T="${PX4_PARAM_COM_DL_LOSS_T:-10}"
export PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-0}"
export PX4_PARAM_COM_DISARM_PRFLT="${PX4_PARAM_COM_DISARM_PRFLT:--1}"
export PX4_PARAM_COM_ARM_BAT_MIN="${PX4_PARAM_COM_ARM_BAT_MIN:--1}"
export PX4_PARAM_BAT_LOW_THR="${PX4_PARAM_BAT_LOW_THR:-0.0}"
export PX4_PARAM_BAT_CRIT_THR="${PX4_PARAM_BAT_CRIT_THR:-0.0}"
export PX4_PARAM_BAT_EMERGEN_THR="${PX4_PARAM_BAT_EMERGEN_THR:-0.0}"
export PX4_PARAM_CBRK_SUPPLY_CHK="${PX4_PARAM_CBRK_SUPPLY_CHK:-894281}"
export PX4_PARAM_COM_LOW_BAT_ACT="${PX4_PARAM_COM_LOW_BAT_ACT:-0}"
export PX4_PARAM_SIM_BAT_ENABLE="${PX4_PARAM_SIM_BAT_ENABLE:-0}"
export PX4_PARAM_MPC_XY_VEL_MAX="${PX4_PARAM_MPC_XY_VEL_MAX:-0.8}"
export PX4_PARAM_MPC_XY_CRUISE="${PX4_PARAM_MPC_XY_CRUISE:-0.8}"
export PX4_PARAM_MPC_ACC_HOR="${PX4_PARAM_MPC_ACC_HOR:-0.8}"
export PX4_PARAM_MPC_ACC_HOR_MAX="${PX4_PARAM_MPC_ACC_HOR_MAX:-1.0}"
export PX4_PARAM_MPC_JERK_AUTO="${PX4_PARAM_MPC_JERK_AUTO:-1.5}"
# In the indoor maze we fly on visual odometry, not GNSS. Letting EKF2 fuse
# both GPS and EV on the ground can trip pre-arm innovation checks and keep the
# vehicle permanently "local position invalid" before takeoff.
if [[ "${PX4_ENABLE_VIO:-1}" != "0" ]]; then
    export PX4_PARAM_EKF2_GPS_CTRL="${PX4_PARAM_EKF2_GPS_CTRL:-0}"
    export PX4_PARAM_EKF2_GPS_CHECK="${PX4_PARAM_EKF2_GPS_CHECK:-0}"
    # Match PX4's vision airframe defaults up front. These are the parameters
    # EKF2 expects for a pure external-vision setup, and two of them
    # (HGT_REF, EV_DELAY) are reboot-required so late `param set` calls do not
    # reliably affect the current boot.
    export PX4_PARAM_EKF2_EV_CTRL="${PX4_PARAM_EKF2_EV_CTRL:-15}"
    export PX4_PARAM_EKF2_HGT_REF="${PX4_PARAM_EKF2_HGT_REF:-3}"
    export PX4_PARAM_EKF2_EV_DELAY="${PX4_PARAM_EKF2_EV_DELAY:-5}"
    export PX4_PARAM_ATT_EXT_HDG_M="${PX4_PARAM_ATT_EXT_HDG_M:-1}"
fi
# PX4 ULog writes add avoidable disk I/O and scheduler jitter in headless
# multi-drone SITL. Override this env var to re-enable logging when needed.
# SDLOG_MODE=-1 disables the logger entirely.
export PX4_PARAM_SDLOG_MODE="${PX4_PARAM_SDLOG_MODE:--1}"

# Clean stale SITL rootfs so stale EEPROM params from a previous run cannot
# override our injected params (e.g. NAV_DLL_ACT != 0 from a prior session).
SITL_ROOTFS="/tmp/px4_sitl_${INSTANCE}"
if [[ -d "${SITL_ROOTFS}" ]]; then
    echo "[PX4-${NS}] Removing stale rootfs: ${SITL_ROOTFS}"
    rm -rf "${SITL_ROOTFS}"
fi

cd "${PX4_DIR}"

echo "[PX4-${NS}] instance=${INSTANCE} model=${MODEL_NAME} rootfs=${BUILD_DIR} xrce_port=${PX4_UXRCE_DDS_PORT}"

# ── Inject custom parameters via stdin pipe ──────────────────────────────────
# The co-process below feeds PX4's interactive shell.
# We wait 5 s for PX4 to finish its boot sequence before sending commands.
# "sleep infinity" keeps the pipe open so PX4 doesn't receive EOF.
{
    sleep 5    # wait for PX4 to fully boot before injecting params (SITL boots in ~3 s)

    # Disable all arming checks that don't apply to headless SITL
    echo "param set COM_ARM_WO_GPS 1"        # allow arming without GPS fix
    echo "param set COM_RC_IN_MODE 4"        # no RC required
    echo "param set NAV_DLL_ACT 0"           # GCS connection not required for arming
    echo "param set COM_DL_LOSS_T 10"        # tolerate 10 s GCS loss before failsafe in flight
    echo "param set CBRK_FLIGHTTERM 121212"  # disable flight termination circuit breaker
    echo "param set COM_ARM_CHK_ESCS 0"      # skip ESC arming check (SITL: no real ESCs)
    echo "param set CBRK_USB_CHK 197848"     # disable USB connected check
    echo "param set COM_CPU_MAX -1"          # disable CPU load check
    echo "param set COM_OF_LOSS_T 10"        # tolerate 10 s OFFBOARD loss before failsafe
                                             # (XRCE-DDS timesync can take a few seconds to reconverge)

    # Leave the simulated magnetometer enabled by default. PX4 otherwise has no
    # yaw source until EV yaw fusion starts, and in this stack that can prevent
    # local position from ever becoming valid on the ground.
    if [[ "${PX4_DISABLE_MAG:-0}" != "0" ]]; then
        echo "param set SYS_HAS_MAG 0"
        echo "param set COM_ARM_MAG_ANG -1"
    fi

    # Adjust hover throttle for our heavier drone.
    # x500_base = 2.0 kg; we add lidar (0.83 kg) + camera (0.05 kg) = 2.88 kg total.
    # Required MPC_THR_HOVER ≈ 0.60 × sqrt(2.88/2.0) ≈ 0.72
    echo "param set MPC_THR_HOVER 0.72"      # hover throttle for 2.88 kg variant

    echo "param save"                        # persist params to SITL EEPROM

    echo "[PX4-${NS}] custom parameters applied." >&2

    # ── Phase 2: VIO observability only ──────────────────────────────────────
    # With the EV parameters applied pre-boot above, this stage is now just a
    # sanity check that the ROS side is actually publishing VIO data.
    if [[ "${PX4_ENABLE_VIO:-1}" != "0" ]]; then
        echo "[PX4-${NS}] Waiting for VIO data on /${NS}/fmu/in/vehicle_visual_odometry ..." >&2
        VIO_DETECTED=0
        for _i in $(seq 1 24); do
            if "${WAIT_FOR_VIO}" \
                "/${NS}/fmu/in/vehicle_visual_odometry" \
                --count 3 \
                --timeout 5.0 > /dev/null 2>&1; then
                VIO_DETECTED=1
                break
            fi
            echo "[PX4-${NS}] VIO not ready (attempt ${_i}/24), retrying in 5 s ..." >&2
            sleep 5
        done

        if [[ ${VIO_DETECTED} -eq 1 ]]; then
            echo "[PX4-${NS}] VIO data confirmed (EKF2 vision params were already applied pre-boot)." >&2
        else
            echo "[PX4-${NS}] WARNING: VIO not detected after 120 s." >&2
        fi
    else
        echo "[PX4-${NS}] VIO fusion skipped (PX4_ENABLE_VIO=0, GPS-only mode)." >&2
    fi

    # Keep the pipe alive — DO NOT send EOF
    sleep infinity

} | "${PX4_BIN}" -i "${INSTANCE}" -d "${BUILD_DIR}/etc"
