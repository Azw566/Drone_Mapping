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
  t=17s  Frontier detectors (needs projected_map from OctoMap)
  t=18s  Auto map saver waits for /mission_complete
  t=30s  Offboard controllers (PX4 ~done booting, EKF2 receiving VIO)
  t=40s  Exploration planners + coordinator + POI manager
  t=45s  ArUco detectors (optional; kept off the hot path by default)
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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
    enable_aruco = LaunchConfiguration('enable_aruco')
    save_map_on_complete = LaunchConfiguration('save_map_on_complete')
    map_output_path = LaunchConfiguration('map_output_path')
    map_save_delay_s = LaunchConfiguration('map_save_delay_s')

    # ── Simulation: Gazebo + spawning + bridge + RSPs ─────────────────────
    headless = LaunchConfiguration('headless')
    simulation = _include('drone_bringup', 'simulation.launch.py',
                          {'use_rviz': use_rviz, 'headless': headless})

    # ── PX4 SITL + MicroXRCE agents ───────────────────────────────────────
    px4 = _include('drone_bringup', 'px4_multi.launch.py')

    # ── LIO-SAM (lidar_enricher + 4 nodes per drone) ─────────────────────
    lio_sam = _include('drone_bringup', 'lio_sam_multi.launch.py')

    # ── Visual odometry bridges + offboard controllers ────────────────────
    vio_bridges = _include('px4_offboard', 'px4_offboard.launch.py', {
        'enable_vio_bridges': 'true',
        'enable_controllers': 'false',
    })
    offboard_controllers = _include('px4_offboard', 'px4_offboard.launch.py', {
        'enable_vio_bridges': 'false',
        'enable_controllers': 'true',
        # The exploration stack flies straight to frontier centroids; there is
        # no maze-aware path planner yet, so staying below the 2 m walls
        # guarantees collisions once exploration starts.
        'hover_alt': '3.0',
        # Keep horizontal motion slow enough for the maze; the planner has no
        # obstacle-aware local pathing, so aggressive setpoint chasing overshoots corners.
        'max_exploring_step_m': '0.08',
        # The ArUco camera is forward-facing; when detectors are enabled we yaw
        # into the active frontier so wall tags pass through the camera frustum.
        'face_goal_yaw': enable_aruco,
    })

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
        'odom_topic':    '/d1/lio_sam/mapping/odometry',
        'min_frontier_size': '20',
        'publish_rate_hz': '0.5',
        'max_frontier_distance_m': '12.0',
        'max_frontier_clusters': '64',
    })
    frontier_d2 = _include('frontier_detector', 'frontier.launch.py', {
        'drone_ns':      'd2',
        'map_topic':     '/d2/projected_map',
        'marker_topic':  '/d2/frontiers/markers',
        'frontier_topic': '/d2/frontiers/list',
        'odom_topic':    '/d2/lio_sam/mapping/odometry',
        'min_frontier_size': '20',
        'publish_rate_hz': '0.5',
        'max_frontier_distance_m': '12.0',
        'max_frontier_clusters': '64',
    })

    # ── ArUco detectors ───────────────────────────────────────────────────
    aruco_d1 = _include('aruco_detector', 'aruco_detector.launch.py',
                        {'drone_ns': 'd1', 'marker_size': '0.6'})
    aruco_d2 = _include('aruco_detector', 'aruco_detector.launch.py',
                        {'drone_ns': 'd2', 'marker_size': '0.6'})

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

    _map_saver_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'auto_save_arena_map.py')
    auto_map_saver = ExecuteProcess(
        cmd=[
            'python3',
            _map_saver_script,
            '--out', map_output_path,
            '--save-delay', map_save_delay_s,
        ],
        name='auto_save_arena_map',
        output='screen',
        condition=IfCondition(save_map_on_complete),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualisation'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only without GUI (auto-detected if no DISPLAY)'),
        DeclareLaunchArgument(
            'enable_aruco',
            default_value='false',
            description='Launch ArUco detectors during mapping/exploration'),
        DeclareLaunchArgument(
            'save_map_on_complete',
            default_value='true',
            description='Auto-save the merged arena map when /mission_complete is published'),
        DeclareLaunchArgument(
            'map_output_path',
            default_value='',
            description='Optional PNG path for the auto-saved arena map'),
        DeclareLaunchArgument(
            'map_save_delay_s',
            default_value='5.0',
            description='Delay between /mission_complete and automatic map export'),

        # Restrict CycloneDDS to loopback to cut multicast jitter
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        # t=0s  — Gazebo world + drone spawning + bridge (internal timers)
        simulation,

        # t=10s — PX4 SITL + XRCE agents (Gazebo + models must be up)
        TimerAction(period=10.0, actions=[px4]),

        # t=12s — LIO-SAM + visual odom bridges + GCS heartbeat
        #         (bridge must be publishing sensor topics; PX4 still booting)
        #         GCS heartbeat starts alongside so PX4 sees a GCS by t=15s param injection
        TimerAction(period=12.0, actions=[lio_sam, vio_bridges, gcs_heartbeat]),

        # t=15s — OctoMap servers
        TimerAction(period=15.0, actions=[octomap_d1, octomap_d2]),

        # t=30s — ArUco detectors (optional; start with offboard so the first
        #         exploration legs can already contribute tag sightings)
        GroupAction(
            condition=IfCondition(enable_aruco),
            actions=[TimerAction(period=30.0, actions=[aruco_d1, aruco_d2])],
        ),

        # t=17s — Frontier detectors (need projected_map from OctoMap)
        TimerAction(period=17.0, actions=[frontier_d1, frontier_d2]),

        # t=18s — Lightweight map saver waiting on /mission_complete
        TimerAction(period=18.0, actions=[auto_map_saver]),

        # t=30s — Offboard controllers
        #   d1 PX4 starts t=10s, params injected t=15s
        #   d2 PX4 starts t=15s (staggered), params injected t=20s
        #   10s margin before offboard controllers start
        TimerAction(period=30.0, actions=[offboard_controllers]),

        # t=40s — Exploration stack (planners + coordinator + POI manager)
        #   Give OctoMap/frontier detection time to stabilise before assigning goals.
        TimerAction(period=40.0, actions=[exploration]),
    ])
