from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    drone_ns = LaunchConfiguration('drone_ns')

    pkg_share = get_package_share_directory('drone_slam')
    lio_params = os.path.join(pkg_share, 'config', 'lio_sam_drone.yaml')

    # ── PointCloud adapter ────────────────────────────────────────────────
    # Input:  /{drone_ns}/points_raw   (from ros_gz_bridge)
    # Output: /{drone_ns}/points_enriched  (ring + time fields added)
    # Relative topics resolve under the node namespace automatically.
    pointcloud_adapter = Node(
        package='drone_slam',
        executable='pointcloud_adapter',
        name='pointcloud_adapter',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'input_topic': 'points_raw',
            'output_topic': 'points_enriched',
            'n_scan': 16,
            'frame_id': 'lidar_link',
            'use_best_effort': True,
        }],
    )

    # ── IMU converter ─────────────────────────────────────────────────────
    # Enforces the correct frame_id on the IMU message for LIO-SAM.
    # Input:  /{drone_ns}/imu/data   (from ros_gz_bridge)
    # Output: /{drone_ns}/imu/data_enriched
    imu_converter = Node(
        package='drone_slam',
        executable='imu_converter',
        name='imu_converter',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'input_topic': 'imu/data',
            'output_topic': 'imu/data_enriched',
            'frame_id': 'imu_link',
            'use_best_effort': True,
        }],
    )

    # ── LIO-SAM nodes ─────────────────────────────────────────────────────
    # Each node is launched under /{drone_ns}/lio_sam namespace.
    # Per-drone absolute topic paths are injected as parameter overrides so
    # LIO-SAM subscribes to the correct enriched topics for whichever drone.
    def lio_node(executable, name):
        return Node(
            package='lio_sam',
            executable=executable,
            name=name,
            namespace=[drone_ns, '/lio_sam'],
            output='screen',
            parameters=[
                lio_params,
                # Override the per-drone topics at launch time.
                {
                    'pointCloudTopic': PythonExpression(
                        ["'/' + '", drone_ns, "' + '/points_enriched'"]
                    ),
                    'imuTopic': PythonExpression(
                        ["'/' + '", drone_ns, "' + '/imu/data_enriched'"]
                    ),
                    'odometryFrame': PythonExpression(
                        ["'", drone_ns, "' + '/odom'"]
                    ),
                    'mapFrame': PythonExpression(
                        ["'", drone_ns, "' + '/map'"]
                    ),
                    'baselinkFrame': PythonExpression(
                        ["'", drone_ns, "' + '/base_link'"]
                    ),
                    'lidarFrame': PythonExpression(
                        ["'", drone_ns, "' + '/lidar_link'"]
                    ),
                },
            ],
        )

    imu_preint  = lio_node('lio_sam_imuPreintegration', 'imuPreintegration')
    image_proj  = lio_node('lio_sam_imageProjection',   'imageProjection')
    feature_ext = lio_node('lio_sam_featureExtraction', 'featureExtraction')
    map_opt     = lio_node('lio_sam_mapOptimization',   'mapOptimization')

    return LaunchDescription([
        DeclareLaunchArgument('drone_ns', default_value='d1',
                              description='Drone namespace (d1 or d2)'),
        pointcloud_adapter,
        imu_converter,
        imu_preint,
        image_proj,
        feature_ext,
        map_opt,
    ])
