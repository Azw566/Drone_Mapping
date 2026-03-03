#!/usr/bin/env python3
"""
diagnose_pipeline.py

Samples every critical topic in the pipeline for one drone and reports
which link in the chain is broken. Run this while the full stack is running.

Usage:
  python3 scripts/diagnose_pipeline.py           # check d1 (default)
  python3 scripts/diagnose_pipeline.py --ns d2   # check d2
  python3 scripts/diagnose_pipeline.py --timeout 8
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)

# ── Colours ───────────────────────────────────────────────────────────────────
_G = '\033[32m'; _R = '\033[31m'; _Y = '\033[33m'; _C = '\033[36m'; _E = '\033[0m'
OK   = f'{_G}OK{_E}'
MISS = f'{_R}MISSING{_E}'
WARN = f'{_Y}WARN{_E}'

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
DEFAULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class PipelineDiagNode(Node):
    def __init__(self, ns: str):
        super().__init__('pipeline_diag')
        self._ns = ns
        self._received: dict[str, list] = {}

        checks = [
            # (label, topic, msg_type, qos)
            ('GZ bridge: points_raw',      f'/{ns}/points_raw',                    PointCloud2,         DEFAULT_QOS),
            ('GZ bridge: imu/data',        f'/{ns}/imu/data',                      Imu,                 DEFAULT_QOS),
            ('lidar_enricher: enriched',   f'/{ns}/points_enriched',               PointCloud2,         DEFAULT_QOS),
            ('LIO-SAM odometry',           f'/{ns}/lio_sam/mapping/odometry',      Odometry,            DEFAULT_QOS),
            ('VIO bridge → PX4',           f'/{ns}/fmu/in/vehicle_visual_odometry',VehicleOdometry,     PX4_QOS),
            ('PX4 → VehicleLocalPosition', f'/{ns}/fmu/out/vehicle_local_position',VehicleLocalPosition,PX4_QOS),
            ('PX4 → VehicleStatus',        f'/{ns}/fmu/out/vehicle_status_v1',     VehicleStatus,       PX4_QOS),
            ('Ctrl: OffboardControlMode',  f'/{ns}/fmu/in/offboard_control_mode',  OffboardControlMode, PX4_QOS),
            ('Ctrl: TrajectorySetpoint',   f'/{ns}/fmu/in/trajectory_setpoint',    TrajectorySetpoint,  PX4_QOS),
        ]

        for label, topic, msg_type, qos in checks:
            self._received[label] = []
            self.create_subscription(
                msg_type, topic,
                lambda msg, lbl=label: self._received[lbl].append(msg),
                qos)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns',      default='d1', help='Drone namespace (default: d1)')
    parser.add_argument('--timeout', type=float, default=6.0,
                        help='Seconds to listen per topic group (default: 6)')
    args = parser.parse_args()

    rclpy.init()
    node = PipelineDiagNode(args.ns)

    print(f'\n{"═"*64}')
    print(f'  diagnose_pipeline.py  [ns={args.ns}]')
    print(f'  Listening for {args.timeout}s …')
    print(f'{"═"*64}\n')

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    # ── Report ────────────────────────────────────────────────────────────────
    print(f'  {"Stage":<38} {"Msgs":>5}  Status')
    print(f'  {"─"*38}  {"─"*5}  {"─"*30}')

    first_miss = None
    for label, msgs in node._received.items():
        count  = len(msgs)
        status = OK if count > 0 else MISS
        detail = ''

        # Add extra context for key messages
        if count > 0:
            last = msgs[-1]

            if isinstance(last, VehicleLocalPosition):
                xy_ok = getattr(last, 'xy_valid', False)
                z_ok  = getattr(last, 'z_valid',  False)
                detail = f'xy_valid={xy_ok}  z_valid={z_ok}  pos=({last.x:.2f},{last.y:.2f},{last.z:.2f})'
                if not (xy_ok and z_ok):
                    status = WARN

            elif isinstance(last, VehicleStatus):
                nav  = getattr(last, 'nav_state',    -1)
                arm  = getattr(last, 'arming_state', -1)
                # nav_state 14 = OFFBOARD; arming_state 2 = ARMED
                detail = f'nav_state={nav}  arming_state={arm}'
                if nav != 14:
                    status = WARN

            elif isinstance(last, OffboardControlMode):
                detail = (f'pos={last.position}  vel={last.velocity}')

            elif isinstance(last, TrajectorySetpoint):
                p = getattr(last, 'position', [None]*3)
                detail = f'pos=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})'

            elif isinstance(last, VehicleOdometry):
                pos = getattr(last, 'position', [0,0,0])
                detail = f'pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})'

        print(f'  {label:<38} {count:>5}  {status}  {detail}')

        if count == 0 and first_miss is None:
            first_miss = label

    # ── Diagnosis ────────────────────────────────────────────────────────────
    print()
    if first_miss is None:
        print(f'{_G}All pipeline stages are receiving data.{_E}')
        print('Check VehicleStatus nav_state (want 14=OFFBOARD) and VehicleLocalPosition xy/z_valid.')
    else:
        print(f'{_R}Pipeline blocked at: {first_miss}{_E}')
        hints = {
            'GZ bridge: points_raw':
                '→ ros_gz_bridge not running, wrong GZ topic name in bridge.yaml,\n'
                '   or Gazebo model not spawned yet.\n'
                '   Check: gz topic -l | grep vlp16',
            'GZ bridge: imu/data':
                '→ Bridge config missing IMU entry or Gazebo IMU sensor not active.',
            'lidar_enricher: enriched':
                '→ lidar_enricher node not running or crashing.\n'
                '   Check: ros2 node list | grep enricher\n'
                '          ros2 run drone_bringup lidar_enricher.py --ros-args -p input_topic:=/d1/points_raw',
            'LIO-SAM odometry':
                '→ LIO-SAM not producing output. Common causes:\n'
                '   - ring/time fields missing or misaligned in points_enriched\n'
                '   - IMU frame mismatch (check lio_sam_d1.yaml imuTopic)\n'
                '   - LIO-SAM nodes crashed: ros2 node list | grep lio_sam\n'
                '   - Check: ros2 topic hz /d1/points_enriched',
            'VIO bridge → PX4':
                '→ visual_odom_bridge not running, or LIO-SAM odometry (above) missing.',
            'PX4 → VehicleLocalPosition':
                '→ MicroXRCE-DDS agent not connecting to PX4, or PX4 not running.\n'
                '   Check: ros2 topic list | grep fmu\n'
                '          ps aux | grep MicroXRCE',
            'PX4 → VehicleStatus':
                '→ Same as VehicleLocalPosition — XRCE bridge or PX4 not running.\n'
                '   Also check topic name: PX4 v1.16 publishes vehicle_status_v1.',
            'Ctrl: OffboardControlMode':
                '→ offboard_controller not running, OR it is stuck in IDLE.\n'
                '   IDLE exit requires xy_valid+z_valid on VehicleLocalPosition.\n'
                '   If VehicleLocalPosition shows xy_valid=False: EKF2 has no fix.\n'
                '   PX4 needs GPS (check Gazebo GPS sensor) or VIO (EKF2_EV_CTRL=15).\n'
                '   Phase-2 VIO params are injected ~40s after PX4 boot — wait longer.',
            'Ctrl: TrajectorySetpoint':
                '→ offboard_controller stuck in IDLE (no position fix) or PRE_ARM.\n'
                '   See OffboardControlMode hint above.',
        }
        hint = hints.get(first_miss, '')
        if hint:
            print(f'\n{_C}Hint:{_E}\n{hint}')

    print()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
