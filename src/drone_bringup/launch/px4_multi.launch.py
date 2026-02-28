"""
px4_multi.launch.py

Starts two PX4 SITL instances (d1 at index 0, d2 at index 1) that attach to
the Gazebo world already started by simulation.launch.py (standalone mode),
plus one MicroXRCE-DDS Agent per instance to bridge PX4 uORB ↔ ROS2.

Gazebo model names must match what simulation.launch.py spawns:
  x500_d1  (connected to PX4 instance 0, namespace d1, uXRCE port 8888)
  x500_d2  (connected to PX4 instance 1, namespace d2, uXRCE port 8889)

ROS2 topics published by PX4 (via uXRCE-DDS):
  /d1/fmu/out/vehicle_status
  /d1/fmu/out/vehicle_local_position
  /d1/fmu/out/vehicle_odometry
  /d1/fmu/in/vehicle_command
  /d1/fmu/in/offboard_control_mode
  /d1/fmu/in/trajectory_setpoint
  /d1/fmu/in/vehicle_visual_odometry
  (equivalent /d2/fmu/* for drone 2)

Environment variables (can be overridden before launching):
  PX4_DIR      path to PX4-Autopilot source tree
               default: /home/telemaque/px4_workspace/PX4-Autopilot
  XRCE_AGENT   path to MicroXRCEAgent binary
               default: /home/telemaque/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

# ── Paths ────────────────────────────────────────────────────────────────────
PX4_DIR = os.path.expanduser(
    os.environ.get('PX4_DIR',
                   '/home/telemaque/px4_workspace/PX4-Autopilot'))

XRCE_AGENT = os.environ.get(
    'XRCE_AGENT',
    '/home/telemaque/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')


def _script_path() -> str:
    """Return the absolute path to launch_px4_instance.sh (installed in lib/)."""
    pkg_prefix = get_package_prefix('drone_bringup')
    return os.path.join(pkg_prefix, 'lib', 'drone_bringup', 'launch_px4_instance.sh')


def _px4_process(instance: int, model: str, ns: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=['bash', _script_path(), str(instance), model, ns],
        name=f'px4_{ns}',
        additional_env={'PX4_DIR': PX4_DIR},
        output='screen',
    )


def _xrce_process(port: int) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[XRCE_AGENT, 'udp4', '-p', str(port)],
        name=f'xrce_agent_port{port}',
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        # ── MicroXRCE-DDS Agents (start first so PX4 can connect) ────────────
        _xrce_process(8888),   # bridges d1 (PX4 instance 0)
        _xrce_process(8889),   # bridges d2 (PX4 instance 1)

        # ── PX4 SITL instances ───────────────────────────────────────────────
        # Stagger d2 by 5 s to avoid a symlink race: both instances would
        # otherwise try to create BUILD_DIR/etc simultaneously and the second
        # one fails with exit code 255.
        _px4_process(0, 'x500_d1', 'd1'),
        TimerAction(period=5.0, actions=[_px4_process(1, 'x500_d2', 'd2')]),
    ])
