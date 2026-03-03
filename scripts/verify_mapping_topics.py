#!/usr/bin/env python3
"""
verify_mapping_topics.py

Live-refreshing pipeline health monitor for the mapping_hover stack.

Subscribes to every topic in the pipeline (for both drones) and prints a
colour-coded table that refreshes every few seconds showing:
  • actual publish rate (Hz) over the last window
  • per-stage pass/warn/fail status
  • extra fields for PX4 topics (arming state, nav state, position validity)

Usage (source workspace first):
  python3 scripts/verify_mapping_topics.py
  python3 scripts/verify_mapping_topics.py --interval 4
  python3 scripts/verify_mapping_topics.py --wait 45   # wait longer before first check
"""

import argparse
import collections
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, Imu
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)

# ── Colours ───────────────────────────────────────────────────────────────────
_G = '\033[32m'
_R = '\033[31m'
_Y = '\033[33m'
_C = '\033[36m'
_B = '\033[1m'
_E = '\033[0m'

OK   = f'{_G}OK  {_E}'
WARN = f'{_Y}WARN{_E}'
MISS = f'{_R}MISS{_E}'
WAIT = f'{_C}WAIT{_E}'

# ── QoS profiles ──────────────────────────────────────────────────────────────
# BEST_EFFORT subscriber is compatible with both BEST_EFFORT and RELIABLE
# publishers (subscriber requests ≤ what publisher offers).
# LIO-SAM, lidar_enricher and PX4 all publish BEST_EFFORT; ros_gz_bridge
# publishes RELIABLE — BEST_EFFORT subscriber works for all of them.
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_PX4_QOS  = _BEST_EFFORT_QOS
_DEF_QOS  = _BEST_EFFORT_QOS

# ── Pipeline definition ───────────────────────────────────────────────────────
# Each entry: (stage_label, key, topic, msg_type, qos, min_hz)
# min_hz: float threshold — below this → WARN; 0 → just presence check
def _pipeline(ns: str) -> list:
    return [
        # Stage 1 — Gazebo bridge
        (f'[{ns}] GZ bridge: points_raw',
         f'{ns}_pts_raw',
         f'/{ns}/points_raw',      PointCloud2,         _DEF_QOS, 5.0),
        (f'[{ns}] GZ bridge: imu/data',
         f'{ns}_imu',
         f'/{ns}/imu/data',        Imu,                 _DEF_QOS, 50.0),
        # Stage 2 — lidar_enricher
        (f'[{ns}] lidar_enricher: enriched',
         f'{ns}_enriched',
         f'/{ns}/points_enriched', PointCloud2,         _DEF_QOS, 5.0),
        # Stage 3 — LIO-SAM
        (f'[{ns}] LIO-SAM: odometry',
         f'{ns}_odom',
         f'/{ns}/lio_sam/mapping/odometry',            Odometry, _DEF_QOS, 3.0),
        (f'[{ns}] LIO-SAM: cloud_registered',
         f'{ns}_cloud',
         f'/{ns}/lio_sam/mapping/cloud_registered',   PointCloud2, _DEF_QOS, 0.5),
        # Stage 4 — VIO bridge
        (f'[{ns}] VIO bridge → PX4',
         f'{ns}_vio',
         f'/{ns}/fmu/in/vehicle_visual_odometry',     VehicleOdometry, _PX4_QOS, 5.0),
        # Stage 5 — PX4 outputs
        (f'[{ns}] PX4: vehicle_local_position',
         f'{ns}_vlp',
         f'/{ns}/fmu/out/vehicle_local_position',     VehicleLocalPosition, _PX4_QOS, 10.0),
        (f'[{ns}] PX4: vehicle_status',
         f'{ns}_vs',
         f'/{ns}/fmu/out/vehicle_status_v1',          VehicleStatus, _PX4_QOS, 1.0),
        # Stage 6 — OctoMap
        (f'[{ns}] OctoMap: projected_map',
         f'{ns}_map',
         f'/{ns}/projected_map',  OccupancyGrid, _DEF_QOS, 0.3),
    ]


# ── Monitor node ──────────────────────────────────────────────────────────────
class MappingMonitor(Node):
    def __init__(self, namespaces: list[str]):
        super().__init__('mapping_monitor')

        # timestamps: key → deque of monotonic times
        self._ts: dict[str, collections.deque] = {}
        # last message cache for extra detail fields
        self._last: dict[str, object] = {}
        # full pipeline entry list
        self._pipeline: list[tuple] = []

        for ns in namespaces:
            for stage, key, topic, msg_type, qos, min_hz in _pipeline(ns):
                self._ts[key] = collections.deque()
                self._pipeline.append((stage, key, topic, msg_type, qos, min_hz))
                self.create_subscription(
                    msg_type, topic,
                    lambda msg, k=key: self._cb(k, msg),
                    qos)

    def _cb(self, key: str, msg) -> None:
        self._ts[key].append(time.monotonic())
        self._last[key] = msg

    def hz(self, key: str, window: float) -> float:
        now = time.monotonic()
        dq = self._ts[key]
        while dq and dq[0] < now - window:
            dq.popleft()
        return len(dq) / window

    def extra(self, key: str) -> str:
        msg = self._last.get(key)
        if msg is None:
            return ''
        if isinstance(msg, VehicleLocalPosition):
            xy = getattr(msg, 'xy_valid', False)
            z  = getattr(msg, 'z_valid',  False)
            ok = xy and z
            flag = f'{_G}✓{_E}' if ok else f'{_Y}✗{_E}'
            return f'xy_valid={flag} z_valid={"✓" if z else "✗"}  pos=({msg.x:.1f},{msg.y:.1f},{msg.z:.1f})'
        if isinstance(msg, VehicleStatus):
            nav  = getattr(msg, 'nav_state',    -1)
            arm  = getattr(msg, 'arming_state', -1)
            nav_s  = f'{_G}OFFBOARD{_E}' if nav == 14 else f'{_Y}{nav}{_E}'
            arm_s  = f'{_G}ARMED{_E}'    if arm == 2  else f'{_Y}{arm}{_E}'
            return f'nav={nav_s}  arm={arm_s}'
        return ''


# ── Topic probe (runs ros2 topic info -v for the first broken topic) ──────────
def _probe_topic(topic: str) -> None:
    """Print publisher count + QoS for `topic` using ros2 topic info."""
    import subprocess
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', '-v', topic],
            capture_output=True, text=True, timeout=4.0)
        lines = result.stdout.strip().splitlines()
        # Show publisher section only (first ~15 lines covers it)
        print(f'\n  {_C}ros2 topic info -v {topic}{_E}')
        for line in lines[:20]:
            print(f'    {line}')
    except Exception as exc:
        print(f'  [probe error: {exc}]')


# ── Display ───────────────────────────────────────────────────────────────────
def _render(node: MappingMonitor, window: float, elapsed: float,
            wait_s: float, iteration: int) -> None:
    os.system('clear')
    print(f'{_B}╔══════════════════════════════════════════════════════════════╗{_E}')
    print(f'{_B}║        mapping_hover — pipeline health monitor               ║{_E}')
    print(f'{_B}╚══════════════════════════════════════════════════════════════╝{_E}')
    print(f'  window={window:.0f}s   uptime={elapsed:.0f}s   iter={iteration}   '
          f'Ctrl-C to quit\n')

    if elapsed < wait_s:
        remaining = wait_s - elapsed
        print(f'  {WAIT}  Waiting {remaining:.0f}s for pipeline to boot …')
        print()
        print(f'  Expected start sequence:')
        print(f'    t=12s  LIO-SAM + VIO bridges')
        print(f'    t=15s  OctoMap servers')
        print(f'    t=30s  Offboard controllers (arm → hover)')
        return

    col_stage = 38
    col_hz    = 8
    hdr = (f'  {"Stage":<{col_stage}}  {"Hz":>{col_hz}}  {"Min":>6}  '
           f'{"Status"}  {"Detail"}')
    print(hdr)
    print('  ' + '─' * (len(hdr) + 10))

    prev_ns = None
    for stage, key, topic, _mtype, _qos, min_hz in node._pipeline:
        # Section separator when drone namespace changes
        ns = key.split('_')[0]
        if ns != prev_ns:
            if prev_ns is not None:
                print()
            prev_ns = ns

        actual_hz = node.hz(key, window)
        detail    = node.extra(key)

        if actual_hz == 0.0:
            status = MISS
            hz_str = f'{"—":>{col_hz}}'
        elif actual_hz < min_hz * 0.6:
            status = WARN
            hz_str = f'{actual_hz:>{col_hz}.1f}'
        else:
            status = OK
            hz_str = f'{actual_hz:>{col_hz}.1f}'

        print(f'  {stage:<{col_stage}}  {hz_str}  {min_hz:>6.1f}  {status}  {detail}')

    # Quick summary
    missing = sum(
        1 for _, key, *_ in node._pipeline
        if node.hz(key, window) == 0.0
    )
    warn = sum(
        1 for _, key, _t, _m, _q, min_hz in node._pipeline
        if 0.0 < node.hz(key, window) < min_hz * 0.6
    )
    total = len(node._pipeline)
    ok_count = total - missing - warn

    print()
    if missing == 0 and warn == 0:
        print(f'  {_G}{_B}All {total} stages healthy.{_E}')
    else:
        print(f'  {_G}OK:{ok_count}{_E}  {_Y}WARN:{warn}{_E}  {_R}MISS:{missing}{_E}  '
              f'(of {total} stages)')
        if missing > 0:
            # Find first missing topic and probe it with ros2 topic info
            for stage, key, first_miss_topic, _m, _q, _hz in node._pipeline:
                if node.hz(key, window) == 0.0:
                    print(f'\n  {_R}Pipeline broken at:{_E} {stage}')
                    _probe_topic(first_miss_topic)
                    break
    print()


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Live pipeline health monitor for mapping_hover stack')
    parser.add_argument('--interval', type=float, default=3.0,
                        help='Refresh interval in seconds (default: 3)')
    parser.add_argument('--wait',     type=float, default=35.0,
                        help='Seconds to wait before checking topics (default: 35)')
    parser.add_argument('--ns',       nargs='+',  default=['d1', 'd2'],
                        help='Drone namespaces to monitor (default: d1 d2)')
    args = parser.parse_args()

    rclpy.init()
    node = MappingMonitor(args.ns)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t0      = time.monotonic()
    window  = max(args.interval, 2.0)   # Hz window = refresh interval (min 2s)
    iteration = 0

    try:
        while True:
            elapsed = time.monotonic() - t0
            _render(node, window, elapsed, args.wait, iteration)
            iteration += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
