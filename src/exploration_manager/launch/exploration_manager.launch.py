"""
exploration_manager.launch.py

Launches the complete exploration intelligence stack:
  - exploration_planner_d1  — selects/tracks frontier goal for drone d1
  - exploration_planner_d2  — same for d2
  - drone_coordinator       — assigns non-overlapping frontiers to idle drones
  - poi_manager             — deduplicates ArUco tag sightings from all drones
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _planner(ns: str,
             goal_radius: LaunchConfiguration,
             search_goal_radius: LaunchConfiguration,
             inspect_at_goal: LaunchConfiguration,
             goal_reached_dwell_s: LaunchConfiguration) -> Node:
    return Node(
        package='exploration_manager',
        executable='exploration_planner',
        name=f'exploration_planner_{ns}',
        parameters=[{
            'drone_ns': ns,
            'goal_radius': goal_radius,
            'search_goal_radius': search_goal_radius,
            'hover_alt': 3.0,
            'hover_ready_tolerance_m': 0.5,
            'tick_period_s': 0.5,
            'state_publish_period_s': 1.0,
            'goal_republish_period_s': 2.0,
            'inspect_at_goal': inspect_at_goal,
            'goal_reached_dwell_s': goal_reached_dwell_s,
        }],
        output='screen',
    )


def generate_launch_description():
    goal_radius = LaunchConfiguration('goal_radius')
    search_goal_radius = LaunchConfiguration('search_goal_radius')
    inspect_at_goal = LaunchConfiguration('inspect_at_goal')
    goal_reached_dwell_s = LaunchConfiguration('goal_reached_dwell_s')
    enable_poi_search = LaunchConfiguration('enable_poi_search')
    required_tag_ids = LaunchConfiguration('required_tag_ids')
    poi_search_targets = LaunchConfiguration('poi_search_targets')
    poi_search_cooldown_s = LaunchConfiguration('poi_search_cooldown_s')
    poi_search_leg_timeout_s = LaunchConfiguration('poi_search_leg_timeout_s')
    poi_search_preempts_frontiers = LaunchConfiguration('poi_search_preempts_frontiers')
    drone_spawn_positions = LaunchConfiguration('drone_spawn_positions')
    poi_reference_tag_positions = LaunchConfiguration('poi_reference_tag_positions')
    max_tag_offset_refine_delta_m = LaunchConfiguration('max_tag_offset_refine_delta_m')
    enable_interest_points = LaunchConfiguration('enable_interest_points')
    interest_points = LaunchConfiguration('interest_points')
    interest_points_preempt_frontiers = LaunchConfiguration('interest_points_preempt_frontiers')
    interest_point_cooldown_s = LaunchConfiguration('interest_point_cooldown_s')
    require_interest_points_for_completion = LaunchConfiguration(
        'require_interest_points_for_completion')

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_radius',
            default_value='0.8',
            description='Goal tolerance for frontier completion'),
        DeclareLaunchArgument(
            'search_goal_radius',
            default_value='0.8',
            description='Goal tolerance for explicit tag-search waypoints'),
        DeclareLaunchArgument(
            'inspect_at_goal',
            default_value='false',
            description='Hold each reached frontier for a short tag inspection scan'),
        DeclareLaunchArgument(
            'goal_reached_dwell_s',
            default_value='8.0',
            description='Inspection dwell time once a frontier has been reached'),
        DeclareLaunchArgument(
            'enable_poi_search',
            default_value='false',
            description='Use dedicated scan waypoints to finish missing ArUco tags'),
        DeclareLaunchArgument(
            'required_tag_ids',
            default_value='',
            description='Comma-separated ArUco tag ids required before mission complete'),
        DeclareLaunchArgument(
            'poi_search_targets',
            default_value='',
            description='Semicolon-separated tag:x:y scan points for ArUco search'),
        DeclareLaunchArgument(
            'poi_search_cooldown_s',
            default_value='30.0',
            description='Minimum delay before reusing the same ArUco scan point'),
        DeclareLaunchArgument(
            'poi_search_leg_timeout_s',
            default_value='45.0',
            description='Time before a POI search leg rotates to another viewpoint'),
        DeclareLaunchArgument(
            'poi_search_preempts_frontiers',
            default_value='false',
            description='Prioritize missing-tag scan waypoints before frontier assignments'),
        DeclareLaunchArgument(
            'drone_spawn_positions',
            default_value='',
            description='Semicolon-separated ns:x:y world positions used to bootstrap map offsets'),
        DeclareLaunchArgument(
            'poi_reference_tag_positions',
            default_value='',
            description='Semicolon-separated tag:x:y world positions for refining POI map offsets'),
        DeclareLaunchArgument(
            'max_tag_offset_refine_delta_m',
            default_value='1.5',
            description='Reject ArUco-derived map-offset corrections larger than this distance'),
        DeclareLaunchArgument(
            'enable_interest_points',
            default_value='false',
            description='Assign fixed scan waypoints after mapping or alongside frontiers'),
        DeclareLaunchArgument(
            'interest_points',
            default_value='',
            description='Semicolon-separated label:x:y:z scan points in world coords; coordinator shifts them into each drone map frame'),
        DeclareLaunchArgument(
            'interest_points_preempt_frontiers',
            default_value='false',
            description='Prioritize fixed interest-point scans before frontier work'),
        DeclareLaunchArgument(
            'interest_point_cooldown_s',
            default_value='10.0',
            description='Minimum delay before retrying the same fixed interest point'),
        DeclareLaunchArgument(
            'require_interest_points_for_completion',
            default_value='false',
            description='Require every fixed interest point to be visited before mission complete'),
        # ── Per-drone planners ────────────────────────────────────────────
        TimerAction(period=0.0, actions=[_planner('d1', goal_radius, search_goal_radius, inspect_at_goal, goal_reached_dwell_s)]),
        TimerAction(period=2.0, actions=[_planner('d2', goal_radius, search_goal_radius, inspect_at_goal, goal_reached_dwell_s)]),

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
                    'enable_poi_search': enable_poi_search,
                    'required_tag_ids': required_tag_ids,
                    'poi_search_targets': poi_search_targets,
                    'poi_search_cooldown_s': poi_search_cooldown_s,
                    'poi_search_leg_timeout_s': poi_search_leg_timeout_s,
                    'poi_search_preempts_frontiers': poi_search_preempts_frontiers,
                    'drone_spawn_positions': drone_spawn_positions,
                    'poi_reference_tag_positions': poi_reference_tag_positions,
                    'max_tag_offset_refine_delta_m': max_tag_offset_refine_delta_m,
                    'enable_interest_points': enable_interest_points,
                    'interest_points': interest_points,
                    'interest_points_preempt_frontiers': interest_points_preempt_frontiers,
                    'interest_point_cooldown_s': interest_point_cooldown_s,
                    'require_interest_points_for_completion': require_interest_points_for_completion,
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
