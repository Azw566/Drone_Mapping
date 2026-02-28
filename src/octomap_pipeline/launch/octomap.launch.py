from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    drone_ns = LaunchConfiguration('drone_ns')
    cloud_topic = LaunchConfiguration('cloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    resolution = LaunchConfiguration('resolution')

    pkg_share = get_package_share_directory('octomap_pipeline')
    default_params = os.path.join(pkg_share, 'params', 'octomap_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_ns',
            default_value='d1',
            description='Drone namespace (d1 or d2) — used to prefix output topics'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/d1/lio_sam/mapping/cloud_registered',
            description='PointCloud2 topic from LIO-SAM mapOptimization'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='d1/map',
            description='Fixed frame matching LIO-SAM mapFrame (e.g. d1/map)'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.1',
            description='OctoMap voxel resolution in metres'),

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
                # Input: LIO-SAM registered point cloud
                ('cloud_in', cloud_topic),
                # Outputs: namespace under the drone so d1 and d2 can coexist
                ('projected_map',            ['/', drone_ns, '/projected_map']),
                ('octomap_binary',           ['/', drone_ns, '/octomap_binary']),
                ('octomap_full',             ['/', drone_ns, '/octomap_full']),
                ('occupied_cells_vis_array', ['/', drone_ns, '/occupied_cells_vis_array']),
                ('free_cells_vis_array',     ['/', drone_ns, '/free_cells_vis_array']),
            ],
        ),
    ])
