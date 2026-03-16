"""
px4_offboard.launch.py

Launches per-drone nodes:
  - visual_odom_bridge : converts LIO-SAM ENU odometry → PX4 NED VehicleOdometry
  - offboard_controller: arm / takeoff / navigate to exploration goals

One launch file handles both drones. Each node is parametrized with drone_ns.
Launch arguments allow the VIO bridges and controllers to be scheduled
independently by a parent launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _vio_bridge(ns: str, enabled: LaunchConfiguration) -> Node:
    return Node(
        package='px4_offboard',
        executable='visual_odom_bridge',
        name=f'visual_odom_bridge_{ns}',
        parameters=[{'drone_ns': ns}],
        condition=IfCondition(enabled),
        output='screen',
    )


def _controller(ns: str, hover_alt: LaunchConfiguration,
                max_exploring_step_m: LaunchConfiguration,
                face_goal_yaw: LaunchConfiguration,
                enabled: LaunchConfiguration) -> Node:
    return Node(
        package='px4_offboard',
        executable='offboard_controller',
        name=f'offboard_controller_{ns}',
        parameters=[{
            'drone_ns': ns,
            'hover_alt': hover_alt,
            'max_exploring_step_m': max_exploring_step_m,
            'face_goal_yaw': face_goal_yaw,
        }],
        condition=IfCondition(enabled),
        output='screen',
    )


def generate_launch_description():
    enable_vio_bridges = LaunchConfiguration('enable_vio_bridges')
    enable_controllers = LaunchConfiguration('enable_controllers')
    hover_alt = LaunchConfiguration('hover_alt')
    max_exploring_step_m = LaunchConfiguration('max_exploring_step_m')
    face_goal_yaw = LaunchConfiguration('face_goal_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vio_bridges',
            default_value='true',
            description='Launch visual_odom_bridge nodes for both drones'),
        DeclareLaunchArgument(
            'enable_controllers',
            default_value='true',
            description='Launch offboard_controller nodes for both drones'),
        DeclareLaunchArgument(
            'hover_alt',
            default_value='1.5',
            description='Hover altitude in metres for offboard controllers'),
        DeclareLaunchArgument(
            'max_exploring_step_m',
            default_value='0.08',
            description='Maximum horizontal setpoint step per controller tick'),
        DeclareLaunchArgument(
            'face_goal_yaw',
            default_value='false',
            description='Yaw toward the active exploration goal'),

        _vio_bridge('d1', enable_vio_bridges),
        _vio_bridge('d2', enable_vio_bridges),
        _controller('d1', hover_alt, max_exploring_step_m, face_goal_yaw, enable_controllers),
        _controller('d2', hover_alt, max_exploring_step_m, face_goal_yaw, enable_controllers),
    ])
