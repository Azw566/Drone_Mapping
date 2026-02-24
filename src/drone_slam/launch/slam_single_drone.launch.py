from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    drone_ns = LaunchConfiguration('drone_ns')

    pkg_share = get_package_share_directory('drone_slam')
    lio_params = os.path.join(pkg_share, 'config', 'lio_sam_drone.yaml')

    pointcloud_adapter = Node(
        package='drone_slam',
        executable='pointcloud_adapter',
        name='pointcloud_adapter',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'input_topic': 'points_raw',
            'output_topic': 'points_adapted',
            'n_scan': 16,
            'frame_id': 'lidar_link',
            'use_best_effort': True,
        }],
    )

    imu_converter = Node(
        package='drone_slam',
        executable='imu_converter',
        name='imu_converter',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'input_topic': 'imu/data',
            'output_topic': 'imu/data_adapted',
            'frame_id': 'imu_link',
            'use_best_effort': True,
        }],
    )

    imu_preint = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='imuPreintegration',
        namespace=[drone_ns, '/lio_sam'],
        output='screen',
        parameters=[lio_params],
    )

    image_proj = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='imageProjection',
        namespace=[drone_ns, '/lio_sam'],
        output='screen',
        parameters=[lio_params],
    )

    feature_ext = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='featureExtraction',
        namespace=[drone_ns, '/lio_sam'],
        output='screen',
        parameters=[lio_params],
    )

    map_opt = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='mapOptimization',
        namespace=[drone_ns, '/lio_sam'],
        output='screen',
        parameters=[lio_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_ns', default_value='d1'),
        pointcloud_adapter,
        imu_converter,
        imu_preint,
        image_proj,
        feature_ext,
        map_opt,
    ])
