"""
full_stack_simple.launch.py

Actual simplified end-to-end stack:
  - Gazebo ceiling maze
  - bridged lidar, camera, and odometry
  - direct model repositioning for a fixed survey
  - optional fixed scan points near wall-mounted ArUco tags
  - OctoMap for each drone
  - optional ArUco detection + POI merge
  - merged map saved after a fixed timeout

This intentionally avoids PX4, EKF2, LIO-SAM, VIO, and frontier logic.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, launch_args=None):
    pkg_dir = get_package_share_directory(pkg)
    src = PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', launch_file))
    return IncludeLaunchDescription(src, launch_arguments=(launch_args or {}).items())


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    survey_alt = LaunchConfiguration('survey_alt')
    route_hold_s = LaunchConfiguration('route_hold_s')
    interest_dwell_s = LaunchConfiguration('interest_dwell_s')
    enable_aruco = LaunchConfiguration('enable_aruco')
    marker_size = LaunchConfiguration('marker_size')
    map_output_path = LaunchConfiguration('map_output_path')
    map_save_timeout_s = LaunchConfiguration('map_save_timeout_s')

    interest_points = (
        'tag0_w:-8.2:-7.5:2.6;'
        'tag0_w_close:-8.8:-7.5:2.1;'
        'tag1_e:8.2:-7.5:2.6;'
        'tag2_nw:-7.5:9.8:2.0;'
        'tag2_nw_close:-7.5:8.7:1.9;'
        'tag3_ne:7.5:9.8:2.0;'
        'tag4_w:-2.6:2.5:2.0;'
        'tag4_w_close:-1.4:2.5:1.8'
    )

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

    tf_d1 = Node(
        package='drone_bringup',
        executable='gz_pose_to_tf.py',
        name='gz_pose_to_tf_d1',
        parameters=[{
            'odom_topic': '/d1/odom',
            'parent_frame': 'd1/map',
            'child_frame': 'd1/base_link',
        }],
        output='screen',
    )
    tf_d2 = Node(
        package='drone_bringup',
        executable='gz_pose_to_tf.py',
        name='gz_pose_to_tf_d2',
        parameters=[{
            'odom_topic': '/d2/odom',
            'parent_frame': 'd2/map',
            'child_frame': 'd2/base_link',
        }],
        output='screen',
    )

    aruco_d1 = _include('aruco_detector', 'aruco_detector.launch.py', {
        'drone_ns': 'd1',
        'marker_size': marker_size,
        'min_confidence': '0.01',
        'max_detection_distance_m': '12.0',
    })
    aruco_d2 = _include('aruco_detector', 'aruco_detector.launch.py', {
        'drone_ns': 'd2',
        'marker_size': marker_size,
        'min_confidence': '0.01',
        'max_detection_distance_m': '12.0',
    })
    poi_manager = Node(
        package='exploration_manager',
        executable='poi_manager',
        name='poi_manager',
        output='screen',
        condition=IfCondition(enable_aruco),
    )

    patrol_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'simple_mapping_patrol.py')
    patrol = ExecuteProcess(
        cmd=[
            'python3', patrol_script,
            '--hold-s', route_hold_s,
            '--altitude-m', survey_alt,
            '--interest-points', interest_points,
            '--interest-dwell-s', interest_dwell_s,
            '--scan-steps', '8',
        ],
        name='simple_mapping_patrol',
        output='screen',
    )

    map_saver_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'save_arena_map.py')
    map_saver = ExecuteProcess(
        cmd=[
            'python3', map_saver_script,
            '--out', map_output_path,
            '--timeout', map_save_timeout_s,
        ],
        name='simple_mapping_map_saver',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('survey_alt', default_value='2.8'),
        DeclareLaunchArgument('route_hold_s', default_value='6.0'),
        DeclareLaunchArgument('interest_dwell_s', default_value='1.25'),
        DeclareLaunchArgument('enable_aruco', default_value='true'),
        DeclareLaunchArgument('marker_size', default_value='0.6'),
        DeclareLaunchArgument('map_output_path', default_value='/tmp/full_stack_simple_map.png'),
        DeclareLaunchArgument('map_save_timeout_s', default_value='180.0'),

        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>',
        ),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        simulation,
        TimerAction(period=12.0, actions=[points_d1, points_d2, tf_d1, tf_d2]),
        TimerAction(period=16.0, actions=[octomap_d1, octomap_d2]),
        TimerAction(period=18.0, actions=[map_saver]),
        TimerAction(period=18.0, actions=[poi_manager], condition=IfCondition(enable_aruco)),
        TimerAction(period=20.0, actions=[aruco_d1, aruco_d2], condition=IfCondition(enable_aruco)),
        TimerAction(period=24.0, actions=[patrol]),
    ])
