from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    resolution = LaunchConfiguration('resolution')

    pkg_share = get_package_share_directory('octomap_pipeline')
    default_params = os.path.join(pkg_share, 'params', 'octomap_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/d1/lio_sam/mapping/cloud_registered',
            description='PointCloud2 topic feeding OctoMap'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Fixed frame for octomap_server output'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.1',
            description='OctoMap resolution (meters)'),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[default_params, {
                'frame_id': frame_id,
                'resolution': resolution,
            }],
            remappings=[
                ('/cloud_in', cloud_topic),
            ],
        ),
    ])
