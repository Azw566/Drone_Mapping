from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_ns       = LaunchConfiguration('drone_ns')
    map_topic      = LaunchConfiguration('map_topic')
    marker_topic   = LaunchConfiguration('marker_topic')
    frontier_topic = LaunchConfiguration('frontier_topic')
    odom_topic     = LaunchConfiguration('odom_topic')
    min_size       = LaunchConfiguration('min_frontier_size')
    publish_rate   = LaunchConfiguration('publish_rate_hz')
    max_distance   = LaunchConfiguration('max_frontier_distance_m')
    max_clusters   = LaunchConfiguration('max_frontier_clusters')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_ns',
            default_value='d1',
            description='Drone namespace'),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/d1/projected_map',
            description='OccupancyGrid from octomap_server'),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/d1/frontiers/markers',
            description='MarkerArray output topic'),
        DeclareLaunchArgument(
            'frontier_topic',
            default_value='/d1/frontiers/list',
            description='FrontierList output topic consumed by the coordinator'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/d1/lio_sam/mapping/odometry',
            description='Odometry used to keep frontier search local to the drone'),
        DeclareLaunchArgument(
            'min_frontier_size',
            default_value='20',
            description='Minimum frontier cluster size (cells)'),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='0.5',
            description='Frontier detector scan/publish rate'),
        DeclareLaunchArgument(
            'max_frontier_distance_m',
            default_value='12.0',
            description='Only scan frontiers within this radius of the drone (0 = full map)'),
        DeclareLaunchArgument(
            'max_frontier_clusters',
            default_value='64',
            description='Maximum number of frontier clusters to publish'),

        Node(
            package='frontier_detector',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic':         map_topic,
                'output_markers':    marker_topic,
                'output_frontiers':  frontier_topic,
                'odom_topic':        odom_topic,
                'min_frontier_size': min_size,
                'publish_rate_hz':   publish_rate,
                'max_frontier_distance_m': max_distance,
                'max_frontier_clusters': max_clusters,
                'use_sim_time':      True,
            }],
        ),
    ])
