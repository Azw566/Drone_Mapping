from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_topic = LaunchConfiguration('map_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    min_size = LaunchConfiguration('min_frontier_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic'),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/frontiers/markers',
            description='MarkerArray output topic'),
        DeclareLaunchArgument(
            'min_frontier_size',
            default_value='5',
            description='Minimum number of frontier cells to publish'),

        Node(
            package='frontier_detector',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic': map_topic,
                'output_markers': marker_topic,
                'min_frontier_size': min_size,
            }],
        ),
    ])
