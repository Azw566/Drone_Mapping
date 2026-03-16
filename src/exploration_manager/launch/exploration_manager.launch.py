"""
exploration_manager.launch.py

Launches the complete exploration intelligence stack:
  - exploration_planner_d1  — selects/tracks frontier goal for drone d1
  - exploration_planner_d2  — same for d2
  - drone_coordinator       — assigns non-overlapping frontiers to idle drones
  - poi_manager             — deduplicates ArUco tag sightings from all drones
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def _planner(ns: str) -> Node:
    return Node(
        package='exploration_manager',
        executable='exploration_planner',
        name=f'exploration_planner_{ns}',
        parameters=[{
            'drone_ns': ns,
            'goal_radius': 0.8,
            'hover_alt': 3.0,
            'hover_ready_tolerance_m': 0.5,
            'tick_period_s': 0.5,
            'state_publish_period_s': 1.0,
            'goal_republish_period_s': 2.0,
        }],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        # ── Per-drone planners ────────────────────────────────────────────
        TimerAction(period=0.0, actions=[_planner('d1')]),
        TimerAction(period=2.0, actions=[_planner('d2')]),

        # ── Singleton coordinator ─────────────────────────────────────────
        TimerAction(
            period=4.0,
            actions=[Node(
                package='exploration_manager',
                executable='drone_coordinator',
                name='drone_coordinator',
                parameters=[{
                    'drone_namespaces': ['d1', 'd2'],
                    'safety_radius': 3.0,
                    'no_frontier_timeout': 120.0,
                    'max_assignment_distance': 12.0,
                    'min_assignment_distance': 1.5,
                    'idle_completion_dwell_s': 5.0,
                }],
                output='screen',
            )],
        ),

        # ── Singleton POI manager ─────────────────────────────────────────
        TimerAction(
            period=6.0,
            actions=[Node(
                package='exploration_manager',
                executable='poi_manager',
                name='poi_manager',
                parameters=[{
                    'drone_namespaces': ['d1', 'd2'],
                }],
                output='screen',
            )],
        ),
    ])
