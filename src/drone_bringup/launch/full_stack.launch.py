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
  t=32s  PX4 SITL instances + MicroXRCE agents
  t=34s  LIO-SAM + visual_odom_bridges + GCS heartbeat
  t=40s  OctoMap servers
  t=43s  Frontier detectors
  t=45s  Auto map saver waits for /mission_complete
  t=52s  Offboard controllers
  t=60s  Exploration planners + coordinator + POI manager
  t=65s  ArUco detectors (optional; kept off the hot path by default)
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
    poi_search_targets = (
        '0:-8.2:-7.5:2.6;'
        '0:-7.2:-7.5:2.6;'
        '1:8.2:-7.5:2.6;'
        '1:7.2:-7.5:2.6;'
        # Tag 2 / 3 normals face north in the maze world, so their search
        # viewpoints need to stay on the north side of the wall.
        '2:-7.5:8.8:2.0;'
        '2:-7.5:9.8:2.0;'
        '2:-6.3:9.2:2.0;'
        '2:-8.7:9.2:2.0;'
        '2:-6.8:10.4:2.0;'
        '2:-8.2:10.4:2.0;'
        '3:7.5:8.8:2.0;'
        '3:7.5:9.8:2.0;'
        '3:6.3:9.2:2.0;'
        '3:8.7:9.2:2.0;'
        '3:6.8:10.4:2.0;'
        '3:8.2:10.4:2.0;'
        # Tag 4 faces west, so bias the search to west-side scan points and
        # descend a bit lower during the scan for a stronger camera angle.
        '4:-2.8:1.8:2.0;'
        '4:-2.6:2.5:2.0;'
        '4:-2.8:3.2:2.0;'
        '4:-1.8:1.6:2.0;'
        '4:-1.8:3.4:2.0'
    )
    drone_spawn_positions = 'd1:-1.0:-8.0;d2:1.0:-8.0'
    poi_reference_tag_positions = (
        '0:-9.89:-7.5;'
        '1:9.89:-7.5;'
        '2:-7.5:7.61;'
        '3:7.5:7.61;'
        '4:-0.11:2.5'
    )

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
        'goal_radius_m': '1.0',
        'search_goal_radius_m': '3.0',
        # Keep horizontal motion slow enough for the maze; the planner has no
        # obstacle-aware local pathing, so aggressive setpoint chasing overshoots corners.
        'max_exploring_step_m': '0.08',
        'inspection_altitude_m': '3.0',
        'force_arm_after_s': '8.0',
        # The ArUco camera is forward-facing; when detectors are enabled we yaw
        # into the active frontier so wall tags pass through the camera frustum.
        'face_goal_yaw': enable_aruco,
        # Give the camera a short in-place scan when a frontier is reached so
        # tags do not depend entirely on the exact approach heading.
        'goal_scan_enabled': enable_aruco,
        'goal_scan_duration_s': '20.0',
        'goal_scan_yawspeed_rad_s': '0.45',
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
                        {
                            'drone_ns': 'd1',
                            'marker_size': '0.6',
                            'min_confidence': '0.01',
                            'max_detection_distance_m': '12.0',
                        })
    aruco_d2 = _include('aruco_detector', 'aruco_detector.launch.py',
                        {
                            'drone_ns': 'd2',
                            'marker_size': '0.6',
                            'min_confidence': '0.01',
                            'max_detection_distance_m': '12.0',
                        })

    # ── Exploration intelligence (planners + coordinator + POI manager) ───
    exploration = _include('exploration_manager', 'exploration_manager.launch.py', {
        'goal_radius': '1.0',
        'search_goal_radius': '3.0',
        'inspect_at_goal': enable_aruco,
        'goal_reached_dwell_s': '20.0',
        'enable_poi_search': enable_aruco,
        'required_tag_ids': '0,1,2,3,4',
        'poi_search_preempts_frontiers': enable_aruco,
        'drone_spawn_positions': drone_spawn_positions,
        'poi_reference_tag_positions': poi_reference_tag_positions,
        'max_tag_offset_refine_delta_m': '1.5',
        # The maze tags are wall-mounted, so once frontier work dries up we
        # deliberately visit a small set of scan viewpoints instead of relying
        # on chance sightings from the mapping path alone.
        'poi_search_targets': poi_search_targets,
        'poi_search_cooldown_s': '12.0',
        'poi_search_leg_timeout_s': '90.0',
    })

    # ── GCS heartbeat (satisfies PX4 'no GCS connection' preflight check) ─
    _heartbeat_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'gcs_heartbeat.py')
    gcs_heartbeat = ExecuteProcess(
        cmd=['python3', _heartbeat_script, '2'],
        name='gcs_heartbeat',
        output='screen',
    )

    _ekf_bootstrap_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'bootstrap_ekf2.py')
    ekf_bootstrap = ExecuteProcess(
        cmd=['python3', _ekf_bootstrap_script, '45'],
        name='ekf2_bootstrapper',
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

        # Let Gazebo finish spawning and sensor startup before PX4 / SLAM join.
        TimerAction(period=32.0, actions=[px4]),

        # Start SLAM and EKF2 bootstrap after the Gazebo model spawns settle.
        # Delay the live VIO bridge a little so PX4 can first lock onto the
        # synthetic bootstrap odometry instead of blending two EV sources
        # while LIO-SAM is still converging.
        TimerAction(period=34.0, actions=[lio_sam, gcs_heartbeat, ekf_bootstrap]),
        TimerAction(period=44.0, actions=[vio_bridges]),

        TimerAction(period=40.0, actions=[octomap_d1, octomap_d2]),

        # t=30s — ArUco detectors (optional; start with offboard so the first
        #         exploration legs can already contribute tag sightings)
        GroupAction(
            condition=IfCondition(enable_aruco),
            actions=[TimerAction(period=65.0, actions=[aruco_d1, aruco_d2])],
        ),

        TimerAction(period=43.0, actions=[frontier_d1, frontier_d2]),

        TimerAction(period=45.0, actions=[auto_map_saver]),

        TimerAction(period=52.0, actions=[offboard_controllers]),

        TimerAction(period=60.0, actions=[exploration]),
    ])
