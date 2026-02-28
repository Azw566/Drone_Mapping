"""
px4_offboard.launch.py

Launches per-drone nodes:
  - visual_odom_bridge : converts LIO-SAM ENU odometry → PX4 NED VehicleOdometry
  - offboard_controller: arm / takeoff / navigate to exploration goals

One launch file handles both drones. Each node is parametrized with drone_ns.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _vio_bridge(ns: str) -> Node:
    return Node(
        package='px4_offboard',
        executable='visual_odom_bridge',
        name=f'visual_odom_bridge_{ns}',
        parameters=[{'drone_ns': ns}],
        output='screen',
    )


def _controller(ns: str) -> Node:
    return Node(
        package='px4_offboard',
        executable='offboard_controller',
        name=f'offboard_controller_{ns}',
        parameters=[{'drone_ns': ns, 'hover_alt': 3.0}],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        _vio_bridge('d1'),
        _vio_bridge('d2'),
        _controller('d1'),
        _controller('d2'),
    ])
