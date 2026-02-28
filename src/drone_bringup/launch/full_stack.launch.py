"""
full_stack.launch.py

Single entry-point for the complete autonomous-exploration pipeline.

Architecture (per drone, e.g. d1)
  Gazebo → ros_gz_bridge → /d1/points_raw  /d1/imu/data  /d1/camera/image_raw
      ↓ lidar_enricher
  /d1/points_enriched
      ↓ LIO-SAM (imuPreintegration · imageProjection · featureExtraction · mapOptimization)
  /d1/lio_sam/mapping/cloud_registered   ← dense registered cloud
  /d1/lio_sam/mapping/odometry           ← global ENU pose
      ↓ visual_odom_bridge               ← ENU → NED for PX4 EKF2
  /d1/fmu/in/vehicle_visual_odometry
      ↓ octomap_server
  /d1/projected_map (OccupancyGrid)
      ↓ frontier_detector
  /d1/frontiers/markers  /d1/frontiers/list (FrontierList)
      ↓ drone_coordinator → exploration_planner → /d1/goal_pose
      ↓ offboard_controller → /d1/fmu/in/trajectory_setpoint
  /d1/camera/image_raw → aruco_detector → /d1/aruco/detections → poi_manager

Start sequence (timers wait for upstream to be ready)
  t=0s   Gazebo world
  t=5s   Spawn drone 1
  t=7s   Spawn drone 2
  t=8s   ros_gz_bridge + robot_state_publishers (inside simulation.launch)
  t=10s  PX4 SITL instances + MicroXRCE agents
  t=12s  LIO-SAM + visual_odom_bridges + GCS heartbeat (needs bridge + PX4 booting)
  t=15s  OctoMap servers  (needs cloud_registered from LIO-SAM)
  t=15s  ArUco detectors  (needs camera topics from bridge)
  t=17s  Frontier detectors (needs projected_map from OctoMap)
  t=20s  Offboard controllers (PX4 ~done booting, EKF2 receiving VIO)
  t=22s  Exploration planners + coordinator + POI manager
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg, launch_file, launch_args=None):
    pkg_dir = get_package_share_directory(pkg)
    src = PythonLaunchDescriptionSource(
        os.path.join(pkg_dir, 'launch', launch_file))
    return IncludeLaunchDescription(src, launch_arguments=(launch_args or {}).items())


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    # ── Simulation: Gazebo + spawning + bridge + RSPs ─────────────────────
    simulation = _include('drone_bringup', 'simulation.launch.py',
                          {'use_rviz': use_rviz})

    # ── PX4 SITL + MicroXRCE agents ───────────────────────────────────────
    px4 = _include('drone_bringup', 'px4_multi.launch.py')

    # ── LIO-SAM (lidar_enricher + 4 nodes per drone) ─────────────────────
    lio_sam = _include('drone_bringup', 'lio_sam_multi.launch.py')

    # ── Visual odometry bridges + offboard controllers ────────────────────
    px4_offboard = _include('px4_offboard', 'px4_offboard.launch.py')

    # ── OctoMap servers ───────────────────────────────────────────────────
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

    # ── Frontier detectors ────────────────────────────────────────────────
    frontier_d1 = _include('frontier_detector', 'frontier.launch.py', {
        'drone_ns':      'd1',
        'map_topic':     '/d1/projected_map',
        'marker_topic':  '/d1/frontiers/markers',
        'frontier_topic': '/d1/frontiers/list',
    })
    frontier_d2 = _include('frontier_detector', 'frontier.launch.py', {
        'drone_ns':      'd2',
        'map_topic':     '/d2/projected_map',
        'marker_topic':  '/d2/frontiers/markers',
        'frontier_topic': '/d2/frontiers/list',
    })

    # ── ArUco detectors ───────────────────────────────────────────────────
    aruco_d1 = _include('aruco_detector', 'aruco_detector.launch.py',
                        {'drone_ns': 'd1'})
    aruco_d2 = _include('aruco_detector', 'aruco_detector.launch.py',
                        {'drone_ns': 'd2'})

    # ── Exploration intelligence (planners + coordinator + POI manager) ───
    exploration = _include('exploration_manager', 'exploration_manager.launch.py')

    # ── GCS heartbeat (satisfies PX4 'no GCS connection' preflight check) ─
    _heartbeat_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'gcs_heartbeat.py')
    gcs_heartbeat = ExecuteProcess(
        cmd=['python3', _heartbeat_script, '2'],
        name='gcs_heartbeat',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualisation'),

        # t=0s  — Gazebo world + drone spawning + bridge (internal timers)
        simulation,

        # t=10s — PX4 SITL + XRCE agents (Gazebo + models must be up)
        TimerAction(period=10.0, actions=[px4]),

        # t=12s — LIO-SAM + visual odom bridges + GCS heartbeat
        #         (bridge must be publishing sensor topics; PX4 still booting)
        #         GCS heartbeat starts alongside so PX4 sees a GCS by t=15s param injection
        TimerAction(period=12.0, actions=[lio_sam, gcs_heartbeat]),

        # t=15s — OctoMap + ArUco detectors
        TimerAction(period=15.0, actions=[octomap_d1, octomap_d2,
                                          aruco_d1,   aruco_d2]),

        # t=17s — Frontier detectors (need projected_map from OctoMap)
        TimerAction(period=17.0, actions=[frontier_d1, frontier_d2]),

        # t=30s — Offboard controllers
        #   d1 PX4 starts t=10s, params injected t=15s
        #   d2 PX4 starts t=15s (staggered), params injected t=20s
        #   10s margin before offboard controllers start
        TimerAction(period=30.0, actions=[px4_offboard]),

        # t=33s — Exploration stack (planners + coordinator + POI manager)
        TimerAction(period=33.0, actions=[exploration]),
    ])
