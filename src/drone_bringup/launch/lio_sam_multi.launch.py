"""
Launch LIO-SAM for d1 and d2 with lidar enricher adapters.
Assumes:
- ros_gz_bridge provides /d1 and /d2 topics (points_raw, imu/data, odom, pose)
- lidar_enricher.py installed in this package
- lio_sam package installed (executable: 'lio_sam')
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def enrich_node(ns, n_scan, horizon_scan, scan_rate):
    return Node(
        package='drone_bringup',
        executable='lidar_enricher.py',
        namespace=ns,
        name=f'lidar_enricher_{ns}',
        parameters=[{
            'input_topic': f'/{ns}/points_raw',
            'output_topic': f'/{ns}/points_enriched',
            'n_scan': n_scan,
            'horizon_scan': horizon_scan,
            'scan_rate': scan_rate,
        }],
        output='screen',
    )


def lio_sam_node(ns, config_path):
    return Node(
        package='lio_sam',
        executable='lio_sam',
        namespace=ns,
        name=f'lio_sam_{ns}',
        parameters=[config_path],
        remappings=[
            ('/lio_sam/imu', f'/{ns}/imu/data'),
            ('/lio_sam/points', f'/{ns}/points_enriched'),
        ],
        output='screen',
    )


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_bringup')
    cfg_d1 = os.path.join(pkg_dir, 'config', 'lio_sam_d1.yaml')
    cfg_d2 = os.path.join(pkg_dir, 'config', 'lio_sam_d2.yaml')

    n_scan = LaunchConfiguration('n_scan')
    horizon_scan = LaunchConfiguration('horizon_scan')
    scan_rate = LaunchConfiguration('scan_rate')

    return LaunchDescription([
        DeclareLaunchArgument('n_scan', default_value='16'),
        DeclareLaunchArgument('horizon_scan', default_value='1800'),
        DeclareLaunchArgument('scan_rate', default_value='10.0'),

        enrich_node('d1', n_scan, horizon_scan, scan_rate),
        lio_sam_node('d1', cfg_d1),

        enrich_node('d2', n_scan, horizon_scan, scan_rate),
        lio_sam_node('d2', cfg_d2),
    ])
