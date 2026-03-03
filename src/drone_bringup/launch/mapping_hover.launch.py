"""
mapping_hover.launch.py

Mapping-only stack: both drones arm, take off, and hover in place while
LIO-SAM builds a SLAM map and OctoMap accumulates a 3-D voxel / 2-D
projected occupancy map.

No exploration, no frontier detection, no ArUco — just the sensing and
mapping pipeline with the drones stationary at hover altitude.

Start sequence
  t=0s   Gazebo world + drone spawning + ros_gz_bridge
  t=2s   Topic-verifier terminal (waits internally for pipeline to boot)
  t=10s  PX4 SITL (x2) + MicroXRCE-DDS agents + GCS heartbeat
  t=12s  lidar_enricher + LIO-SAM (x2) + visual_odom_bridges (x2)
  t=15s  OctoMap servers (x2)
  t=30s  Offboard controllers (arm → takeoff → hover, no exploration goals)
"""

import os
import shutil
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Constants ─────────────────────────────────────────────────────────────────
_ROS_SETUP   = '/opt/ros/humble/setup.bash'
_CYCLONE_URI = (
    '<CycloneDDS><Domain><General>'
    '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
    '</General></Domain></CycloneDDS>'
)


def _include(pkg, launch_file, launch_args=None):
    pkg_dir = get_package_share_directory(pkg)
    src = PythonLaunchDescriptionSource(
        os.path.join(pkg_dir, 'launch', launch_file))
    return IncludeLaunchDescription(src, launch_arguments=(launch_args or {}).items())


def _verifier_terminal(verifier_path: str, ws_root: str,
                        wait_s: float = 35.0, interval_s: float = 3.0):
    """
    Return an ExecuteProcess that opens verify_mapping_topics.py in a new
    terminal window, or None when running headless (no DISPLAY) or when no
    supported terminal emulator is found.
    """
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        return None

    ws_setup = os.path.join(ws_root, 'install', 'setup.bash')
    src = f'source {_ROS_SETUP}'
    if os.path.exists(ws_setup):
        src += f' && source {ws_setup}'

    # Both terminals must share the same CYCLONEDDS_URI (loopback) so that DDS
    # discovery works between simulation processes and the verifier node.
    bash_cmd = (
        f'{src} && '
        f'export CYCLONEDDS_URI=\'{_CYCLONE_URI}\' && '
        f'python3 {verifier_path} --wait {wait_s:.0f} --interval {interval_s:.0f}; '
        f'echo; echo "=== Verifier exited — press Enter to close ==="; read'
    )

    title = 'mapping_hover — topic verifier'

    for binary, build_argv in [
        ('gnome-terminal', lambda: [
            'gnome-terminal', f'--title={title}', '--', 'bash', '-c', bash_cmd]),
        ('xterm', lambda: [
            'xterm', '-title', title, '-fa', 'Monospace', '-fs', '10',
            '-geometry', '160x40', '-e', 'bash', '-c', bash_cmd]),
        ('konsole', lambda: [
            'konsole', '--title', title, '-e', 'bash', '-c', bash_cmd]),
        ('xfce4-terminal', lambda: [
            'xfce4-terminal', f'--title={title}',
            '-e', "bash -c '" + bash_cmd.replace("'", "'\\''") + "'"]),
    ]:
        if shutil.which(binary):
            return ExecuteProcess(
                cmd=build_argv(),
                name='topic_verifier',
                output='log',
            )
    return None


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    headless  = LaunchConfiguration('headless')
    hover_alt = LaunchConfiguration('hover_alt')

    # Workspace root: install/drone_bringup → install → ws_root
    _ws_root        = os.path.normpath(
        os.path.join(get_package_prefix('drone_bringup'), '..', '..'))
    _verifier_script = os.path.join(_ws_root, 'scripts', 'verify_mapping_topics.py')

    # ── Simulation: Gazebo + spawning + bridge + RSPs ─────────────────────
    simulation = _include('drone_bringup', 'simulation.launch.py',
                          {'use_rviz': use_rviz, 'headless': headless})

    # ── PX4 SITL (x2) + MicroXRCE-DDS agents (x2) ────────────────────────
    px4 = _include('drone_bringup', 'px4_multi.launch.py')

    # ── GCS heartbeat ─────────────────────────────────────────────────────
    _heartbeat_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'gcs_heartbeat.py')
    gcs_heartbeat = ExecuteProcess(
        cmd=['python3', _heartbeat_script, '2'],
        name='gcs_heartbeat',
        output='screen',
    )

    # ── LIO-SAM: lidar_enricher + 4 SLAM nodes per drone ─────────────────
    lio_sam = _include('drone_bringup', 'lio_sam_multi.launch.py')

    # ── Visual odometry bridges (LIO-SAM ENU → PX4 NED VIO) ──────────────
    vio_d1 = Node(
        package='px4_offboard',
        executable='visual_odom_bridge',
        name='visual_odom_bridge_d1',
        parameters=[{'drone_ns': 'd1'}],
        output='screen',
    )
    vio_d2 = Node(
        package='px4_offboard',
        executable='visual_odom_bridge',
        name='visual_odom_bridge_d2',
        parameters=[{'drone_ns': 'd2'}],
        output='screen',
    )

    # ── OctoMap servers (build 3-D voxel + 2-D projected map) ─────────────
    octomap_d1 = _include('octomap_pipeline', 'octomap.launch.py', {
        'drone_ns':    'd1',
        'cloud_topic': '/d1/lio_sam/mapping/cloud_registered',
        'frame_id':    'd1/map',
    })
    octomap_d2 = _include('octomap_pipeline', 'octomap.launch.py', {
        'drone_ns':    'd2',
        'cloud_topic': '/d2/lio_sam/mapping/cloud_registered',
        'frame_id':    'd2/map',
    })

    # ── Offboard controllers: arm → takeoff → hover ───────────────────────
    ctrl_d1 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d1',
        parameters=[{'drone_ns': 'd1', 'hover_alt': hover_alt}],
        output='screen',
    )
    ctrl_d2 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d2',
        parameters=[{'drone_ns': 'd2', 'hover_alt': hover_alt}],
        output='screen',
    )

    # ── Topic verifier terminal (skipped when headless / no terminal found) ─
    verifier = _verifier_terminal(_verifier_script, _ws_root,
                                   wait_s=35.0, interval_s=3.0)

    # ── Assemble launch description ────────────────────────────────────────
    actions = [
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 for visualisation'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only without GUI'),
        DeclareLaunchArgument(
            'hover_alt', default_value='1.5',
            description='Hover altitude in metres (AGL)'),

        # Restrict CycloneDDS to loopback to cut multicast jitter
        SetEnvironmentVariable('CYCLONEDDS_URI', _CYCLONE_URI),

        # t=0s  — Gazebo (internal timers: spawn d1@5s, d2@7s, bridge@8s)
        simulation,

        # t=2s  — Topic verifier window (waits 35s internally before first check)
        # Skipped automatically in headless sessions or if no terminal emulator found.
        *([TimerAction(period=2.0, actions=[verifier])] if verifier else []),

        # t=10s — PX4 SITL boots; params injected at t≈15s by launch script
        TimerAction(period=10.0, actions=[px4, gcs_heartbeat]),

        # t=12s — LIO-SAM pipeline + VIO bridges
        TimerAction(period=12.0, actions=[lio_sam, vio_d1, vio_d2]),

        # t=15s — OctoMap servers (need cloud_registered from LIO-SAM)
        TimerAction(period=15.0, actions=[octomap_d1, octomap_d2]),

        # t=30s — Offboard controllers (PX4 fully booted, EKF2 on VIO, ready to arm)
        TimerAction(period=30.0, actions=[ctrl_d1, ctrl_d2]),
    ]

    return LaunchDescription(actions)
