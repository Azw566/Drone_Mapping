"""
simple_mapping.launch.py

Minimal area-mapping stack for the maze:
  - Gazebo ceiling world
  - direct Gazebo pose + raw lidar -> map-frame cloud
  - map-frame cloud -> OctoMap
  - fixed waypoint survey by repositioning the Gazebo models

This intentionally skips PX4, VIO, frontier exploration, and ArUco.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, launch_args=None):
    pkg_dir = get_package_share_directory(pkg)
    src = PythonLaunchDescriptionSource(
        os.path.join(pkg_dir, 'launch', launch_file))
    return IncludeLaunchDescription(src, launch_arguments=(launch_args or {}).items())


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    survey_alt = LaunchConfiguration('survey_alt')
    route_hold_s = LaunchConfiguration('route_hold_s')
    map_output_path = LaunchConfiguration('map_output_path')
    map_save_timeout_s = LaunchConfiguration('map_save_timeout_s')

    simulation = _include('drone_bringup', 'simulation.launch.py', {
        'use_rviz': use_rviz,
        'headless': headless,
    })

    octomap_d1 = _include('octomap_pipeline', 'octomap.launch.py', {
        'drone_ns': 'd1',
        'cloud_topic': '/d1/points_mapped',
        'frame_id': 'd1/map',
    })
    octomap_d2 = _include('octomap_pipeline', 'octomap.launch.py', {
        'drone_ns': 'd2',
        'cloud_topic': '/d2/points_mapped',
        'frame_id': 'd2/map',
    })

    points_d1 = Node(
        package='drone_bringup',
        executable='cloud_to_map.py',
        name='cloud_to_map_d1',
        parameters=[{
            'input_topic': '/d1/points_raw',
            'odom_topic': '/d1/odom',
            'output_topic': '/d1/points_mapped',
            'output_frame_id': 'd1/map',
        }],
        output='screen',
    )
    points_d2 = Node(
        package='drone_bringup',
        executable='cloud_to_map.py',
        name='cloud_to_map_d2',
        parameters=[{
            'input_topic': '/d2/points_raw',
            'odom_topic': '/d2/odom',
            'output_topic': '/d2/points_mapped',
            'output_frame_id': 'd2/map',
        }],
        output='screen',
    )

    _patrol_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'simple_mapping_patrol.py')
    patrol = ExecuteProcess(
        cmd=[
            'python3', _patrol_script,
            '--hold-s', route_hold_s,
            '--altitude-m', survey_alt,
        ],
        name='simple_mapping_patrol',
        output='screen',
    )

    _map_saver_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'save_arena_map.py')
    map_saver = ExecuteProcess(
        cmd=[
            'python3', _map_saver_script,
            '--out', map_output_path,
            '--timeout', map_save_timeout_s,
        ],
        name='simple_mapping_map_saver',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualisation'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only without GUI'),
        DeclareLaunchArgument(
            'survey_alt',
            default_value='2.8',
            description='Survey altitude in metres'),
        DeclareLaunchArgument(
            'route_hold_s',
            default_value='8.0',
            description='Seconds to hold each survey viewpoint'),
        DeclareLaunchArgument(
            'map_output_path',
            default_value='/tmp/simple_mapping_map.png',
            description='Where to save the merged map PNG'),
        DeclareLaunchArgument(
            'map_save_timeout_s',
            default_value='180.0',
            description='How long the map saver should listen before writing the PNG'),

        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        simulation,
        TimerAction(period=12.0, actions=[points_d1, points_d2]),
        TimerAction(period=16.0, actions=[octomap_d1, octomap_d2]),
        TimerAction(period=18.0, actions=[map_saver]),
        TimerAction(period=24.0, actions=[patrol]),
    ])
