"""
lio_sam_multi.launch.py

Launches the full LIO-SAM pipeline (4 nodes) for both drones d1 and d2,
each preceded by a lidar_enricher that adds ring/time fields required by LIO-SAM.

Assumes ros_gz_bridge is already running and publishing:
  /d1/points_raw, /d1/imu/data  (and equivalent /d2/*)

Topic flow per drone (e.g. d1):
  /d1/points_raw
      ↓  lidar_enricher
  /d1/points_enriched
      ↓  lio_sam_imageProjection → lio_sam_featureExtraction
                                         ↓
      lio_sam_imuPreintegration → lio_sam_mapOptimization
                                         ↓
                      /d1/lio_sam/mapping/cloud_registered
                      /d1/lio_sam/mapping/odometry
                      /d1/lio_sam/mapping/path

TF tree published by LIO-SAM:
  mapOptimization  : d1/odom → lidar_link   (hardcoded child frame, at scan rate)
  TransformFusion  : d1/odom → d1/base_link (at IMU rate)

TF tree NOT published by LIO-SAM (mapFrame parameter is declared in utility.hpp
but never used in any sendTransform call in the C++ source):
  d1/map → d1/odom   (MISSING — added here as a static identity transform)

The static map→odom identity transform is correct because LIO-SAM collapses the
map and odometry frames: the odometry poses it publishes are already expressed in
the global (map-origin) frame.  OctoMap and other consumers need the map frame
to exist in the TF tree; without it lookupTransform(d1/map, d1/odom) fails.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def map_to_odom_tf_node(ns):
    """
    Publish a static identity TF: {ns}/map → {ns}/odom.

    LIO-SAM reads 'mapFrame' from the YAML config but the parameter is stored in
    a dead member variable — no sendTransform call in the C++ source ever uses it.
    mapOptimization publishes {ns}/odom → lidar_link (hardcoded bare child frame).
    TransformFusion publishes {ns}/odom → {ns}/base_link.
    Neither node ever broadcasts the map → odom edge, so it must be supplied here.

    The identity translation/rotation is correct because LIO-SAM expresses all
    odometry directly in the map-origin frame (no separate map-correction layer).
    """
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_map_to_odom_{ns}',
        arguments=[
            '--frame-id',       f'{ns}/map',
            '--child-frame-id', f'{ns}/odom',
            # identity: x y z qx qy qz qw
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        ],
        output='screen',
    )


def lidar_enricher_node(ns, n_scan, horizon_scan, scan_rate):
    return Node(
        package='drone_bringup',
        executable='lidar_enricher.py',
        name=f'lidar_enricher_{ns}',
        parameters=[{
            'input_topic':   f'/{ns}/points_raw',
            'output_topic':  f'/{ns}/points_enriched',
            'n_scan':        n_scan,
            'horizon_scan':  horizon_scan,
            'scan_rate':     scan_rate,
        }],
        output='screen',
    )


def lio_sam_nodes(ns, config_path):
    """Return the 4 LIO-SAM nodes for one drone, all sharing the same config."""
    common = dict(
        package='lio_sam',
        namespace=ns,
        parameters=[config_path],
        output='screen',
    )
    return [
        Node(executable='lio_sam_imuPreintegration',
             name='lio_sam_imuPreintegration', **common),
        Node(executable='lio_sam_imageProjection',
             name='lio_sam_imageProjection',   **common),
        Node(executable='lio_sam_featureExtraction',
             name='lio_sam_featureExtraction', **common),
        Node(executable='lio_sam_mapOptimization',
             name='lio_sam_mapOptimization',   **common),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_bringup')
    cfg_d1  = os.path.join(pkg_dir, 'config', 'lio_sam_d1.yaml')
    cfg_d2  = os.path.join(pkg_dir, 'config', 'lio_sam_d2.yaml')

    n_scan       = LaunchConfiguration('n_scan')
    horizon_scan = LaunchConfiguration('horizon_scan')
    scan_rate    = LaunchConfiguration('scan_rate')

    return LaunchDescription([
        DeclareLaunchArgument('n_scan',        default_value='16'),
        DeclareLaunchArgument('horizon_scan',  default_value='1800'),
        DeclareLaunchArgument('scan_rate',     default_value='10.0'),

        # ── Drone 1 ───────────────────────────────────────────────────────
        # Static TF: d1/map → d1/odom (identity).
        # LIO-SAM never publishes this edge; see module docstring above.
        map_to_odom_tf_node('d1'),
        lidar_enricher_node('d1', n_scan, horizon_scan, scan_rate),
        *lio_sam_nodes('d1', cfg_d1),

        # ── Drone 2 ───────────────────────────────────────────────────────
        # Static TF: d2/map → d2/odom (identity).
        map_to_odom_tf_node('d2'),
        lidar_enricher_node('d2', n_scan, horizon_scan, scan_rate),
        *lio_sam_nodes('d2', cfg_d2),
    ])
