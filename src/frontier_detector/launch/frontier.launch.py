from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_ns       = LaunchConfiguration('drone_ns')
    map_topic      = LaunchConfiguration('map_topic')
    marker_topic   = LaunchConfiguration('marker_topic')
    frontier_topic = LaunchConfiguration('frontier_topic')
    min_size       = LaunchConfiguration('min_frontier_size')

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
            'min_frontier_size',
            default_value='5',
            description='Minimum frontier cluster size (cells)'),

        Node(
            package='frontier_detector',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic':         map_topic,
                'output_markers':    marker_topic,
                'output_frontiers':  frontier_topic,
                'min_frontier_size': min_size,
            }],
        ),
    ])
