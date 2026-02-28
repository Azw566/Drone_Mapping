"""
exploration_manager.launch.py

Launches the complete exploration intelligence stack:
  - exploration_planner_d1  — selects/tracks frontier goal for drone d1
  - exploration_planner_d2  — same for d2
  - drone_coordinator       — assigns non-overlapping frontiers to idle drones
  - poi_manager             — deduplicates ArUco tag sightings from all drones
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _planner(ns: str) -> Node:
    return Node(
        package='exploration_manager',
        executable='exploration_planner',
        name=f'exploration_planner_{ns}',
        parameters=[{'drone_ns': ns, 'goal_radius': 0.8}],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        # ── Per-drone planners ────────────────────────────────────────────
        _planner('d1'),
        _planner('d2'),

        # ── Singleton coordinator ─────────────────────────────────────────
        Node(
            package='exploration_manager',
            executable='drone_coordinator',
            name='drone_coordinator',
            parameters=[{
                'drone_namespaces': ['d1', 'd2'],
                'safety_radius': 3.0,
            }],
            output='screen',
        ),

        # ── Singleton POI manager ─────────────────────────────────────────
        Node(
            package='exploration_manager',
            executable='poi_manager',
            name='poi_manager',
            parameters=[{
                'drone_namespaces': ['d1', 'd2'],
            }],
            output='screen',
        ),
    ])
