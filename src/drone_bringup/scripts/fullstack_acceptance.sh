#!/usr/bin/env bash
set -euo pipefail

timeout_s=600
log_path=""
map_output=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)
      timeout_s="$2"
      shift 2
      ;;
    --log)
      log_path="$2"
      shift 2
      ;;
    --map-output)
      map_output="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -z "${log_path}" ]]; then
  log_path="/tmp/fullstack_acceptance_${timestamp}.log"
fi
if [[ -z "${map_output}" ]]; then
  map_output="/tmp/fullstack_acceptance_map_${timestamp}.png"
fi

package_prefix="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if ! command -v ros2 >/dev/null 2>&1 && [[ -f "${package_prefix}/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${package_prefix}/local_setup.bash"
fi

cmd=(
  ros2 launch drone_bringup full_stack.launch.py
  headless:=true
  use_rviz:=false
  enable_aruco:=true
  save_map_on_complete:=true
  map_output_path:="${map_output}"
)

echo "[acceptance] Running: ${cmd[*]}"
echo "[acceptance] Log: ${log_path}"
echo "[acceptance] Map: ${map_output}"

rm -f "${map_output}"
: >"${log_path}"

launch_pid=""
launch_rc=0
success=0

terminate_stack_processes() {
  local patterns=(
    'ros2 launch drone_bringup full_stack.launch.py'
    'launch_px4_instance.sh 0 x500_d1 d1'
    'launch_px4_instance.sh 1 x500_d2 d2'
    'MicroXRCEAgent udp4 -p 8888'
    'MicroXRCEAgent udp4 -p 8889'
    'gz sim -r -v4 -s --headless-rendering .*/worlds/maze.sdf'
    'auto_save_arena_map.py'
  )

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 2
  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
}

kill_launch_group() {
  local signal="$1"
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill "-${signal}" "--" "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}

cleanup() {
  kill_launch_group INT
  terminate_stack_processes
}

trap cleanup EXIT

terminate_stack_processes

setsid stdbuf -oL -eL "${cmd[@]}" >"${log_path}" 2>&1 &
launch_pid=$!
start_s=$SECONDS

while true; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    wait "${launch_pid}" || launch_rc=$?
    break
  fi

  d1_hover=$(grep -c '\[d1\] → HOVER' "${log_path}" || true)
  d2_hover=$(grep -c '\[d2\] → HOVER' "${log_path}" || true)
  d1_exploring=$(grep -c '\[d1\] → EXPLORING' "${log_path}" || true)
  d2_exploring=$(grep -c '\[d2\] → EXPLORING' "${log_path}" || true)
  mission_complete=0
  if grep -q 'Mission complete' "${log_path}"; then
    mission_complete=1
  fi
  mapfile -t found_tags < <(
    grep -oE '\[POI\] New tag [0-9]+' "${log_path}" \
    | awk '{print $4}' \
    | sort -n \
    | uniq
  )

  if [[ "${d1_hover}" -gt 0 && "${d2_hover}" -gt 0 \
        && "${d1_exploring}" -gt 0 && "${d2_exploring}" -gt 0 \
        && "${mission_complete}" -eq 1 \
        && "${#found_tags[@]}" -eq 5 \
        && -s "${map_output}" ]]; then
    success=1
    kill_launch_group INT
    break
  fi

  if (( SECONDS - start_s >= timeout_s )); then
    launch_rc=124
    kill_launch_group INT
    break
  fi

  sleep 5
done

mission_complete=0
if grep -q 'Mission complete' "${log_path}"; then
  mission_complete=1
fi

d1_hover=$(grep -c '\[d1\] → HOVER' "${log_path}" || true)
d2_hover=$(grep -c '\[d2\] → HOVER' "${log_path}" || true)
d1_exploring=$(grep -c '\[d1\] → EXPLORING' "${log_path}" || true)
d2_exploring=$(grep -c '\[d2\] → EXPLORING' "${log_path}" || true)

mapfile -t found_tags < <(
  grep -oE '\[POI\] New tag [0-9]+' "${log_path}" \
  | awk '{print $4}' \
  | sort -n \
  | uniq
)

echo "[acceptance] launch_rc=${launch_rc}"
echo "[acceptance] d1_hover=${d1_hover} d2_hover=${d2_hover}"
echo "[acceptance] d1_exploring=${d1_exploring} d2_exploring=${d2_exploring}"
echo "[acceptance] mission_complete=${mission_complete}"
echo "[acceptance] tags_found=${found_tags[*]:-none}"

failure=0
if [[ "${d1_hover}" -eq 0 || "${d2_hover}" -eq 0 ]]; then
  echo "[acceptance] FAIL: both drones did not reach HOVER" >&2
  failure=1
fi
if [[ "${d1_exploring}" -eq 0 || "${d2_exploring}" -eq 0 ]]; then
  echo "[acceptance] FAIL: both drones did not reach EXPLORING" >&2
  failure=1
fi
if [[ "${mission_complete}" -ne 1 ]]; then
  echo "[acceptance] FAIL: mission did not complete" >&2
  failure=1
fi
if [[ "${#found_tags[@]}" -ne 5 ]]; then
  echo "[acceptance] FAIL: expected 5 unique tags, got ${#found_tags[@]}" >&2
  failure=1
fi
if [[ ! -s "${map_output}" ]]; then
  echo "[acceptance] FAIL: map output missing at ${map_output}" >&2
  failure=1
fi

if [[ "${success}" -eq 1 ]]; then
  echo "[acceptance] PASS"
  exit 0
fi

if [[ "${failure}" -ne 0 ]]; then
  echo "[acceptance] See log: ${log_path}" >&2
  exit 1
fi

echo "[acceptance] PASS"
