"""
arm_takeoff_test.launch.py

Minimal smoke-test for arming and takeoff — no SLAM, no exploration.

Stack:
  t=0s   Gazebo world + drone spawning + bridge  (via simulation.launch.py)
  t=10s  PX4 SITL instances (x2) + MicroXRCE agents + GCS heartbeat
         PX4 uses its built-in SITL GPS → EKF2 initialises without LIO-SAM.
         launch_px4_instance.sh injects arming params 5 s after PX4 boot
         (NAV_DLL_ACT=0, COM_ARM_WO_GPS=1, COM_RC_IN_MODE=4, …)
  t=30s  offboard_controller nodes (d1 + d2)
         State machine: IDLE → PRE_ARM (2 s) → SWITCHING (OFFBOARD mode)
                       → ARMING → TAKING_OFF → HOVER @ 3 m

No visual_odom_bridge needed: EKF2 runs on SITL GPS, not LIO-SAM VIO.
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _pkg_launch(pkg: str, launch_file: str, args: dict | None = None):
    src = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), 'launch', launch_file))
    return IncludeLaunchDescription(src, launch_arguments=(args or {}).items())


def generate_launch_description():
    # ── Gazebo world + drone spawning + ros_gz_bridge ─────────────────────────
    simulation = _pkg_launch('drone_bringup', 'simulation.launch.py',
                             {'use_rviz': 'true'})

    # ── PX4 SITL (x2) + MicroXRCE-DDS agents (x2) ────────────────────────────
    px4 = _pkg_launch('drone_bringup', 'px4_multi.launch.py')

    # ── GCS heartbeat (satisfies the MAVLink GCS-connection preflight check) ──
    _heartbeat = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'gcs_heartbeat.py')
    gcs_heartbeat = ExecuteProcess(
        cmd=['python3', _heartbeat, '2'],
        name='gcs_heartbeat',
        output='screen',
    )

    # ── Offboard controllers (arm + takeoff only, no exploration goals) ───────
    # EKF2 is GPS-based here so xy_valid=True arrives without LIO-SAM.
    # Controllers start in IDLE, wait for xy_valid, then proceed automatically.
    ctrl_d1 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d1',
        parameters=[{'drone_ns': 'd1', 'hover_alt': 3.0}],
        output='screen',
    )
    ctrl_d2 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d2',
        parameters=[{'drone_ns': 'd2', 'hover_alt': 3.0}],
        output='screen',
    )

    return LaunchDescription([
        # Run Gazebo server-only (no GUI) to free ~40% CPU.
        # On a loaded system, gz sim gui prevents physics from running at real-time,
        # which causes PX4 XRCE-DDS timesync to oscillate → OFFBOARD loss → drone lands.
        SetEnvironmentVariable('GZ_HEADLESS', '1'),

        # Restrict CycloneDDS to loopback interface only (system uses CycloneDDS RMW).
        # Eliminates multicast traffic that adds latency jitter to DDS communication.
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>'),

        # t=0s  — Gazebo (internal timers: spawn d1@5s, d2@7s, bridge@8s)
        simulation,

        # t=10s — PX4 SITL boots; params injected at t≈15s (sleep 5 in script)
        TimerAction(period=10.0, actions=[px4, gcs_heartbeat]),

        # t=30s — Offboard controllers (PX4 fully booted, GPS EKF2 valid)
        TimerAction(period=30.0, actions=[ctrl_d1, ctrl_d2]),
    ])
