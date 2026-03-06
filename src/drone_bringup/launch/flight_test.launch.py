"""
flight_test.launch.py

Minimal flight test in an open empty world — no SLAM, no exploration.
Use this to verify that both drones arm, take off, and navigate stably
before running the full stack.

Stack:
  t=0s   Gazebo empty world + drone spawning + ros_gz_bridge
  t=10s  PX4 SITL instances (x2) + MicroXRCE agents + GCS heartbeat
         EKF2 runs on built-in SITL GPS — no LIO-SAM needed.
         launch_px4_instance.sh injects arming params ~5 s after PX4 boot.
  t=30s  offboard_controller nodes (d1 + d2)
         IDLE → PRE_ARM → OFFBOARD → ARMING → TAKING_OFF → HOVER @ hover_alt
  t=50s  Navigation goals published (ENU frame):
           d1 → (goal_x, goal_y)  — flies to the target and holds position
           d2 → (goal_x, goal_y)  — same target (offset by 2 m in Y at spawn)

Drones are spawned at (0, -2) and (0, +2) in the empty world so they have
plenty of clearance and there are no obstacles to interfere with EKF2 GPS.

Launch arguments:
  use_rviz   — launch RViz2 (default: false)
  hover_alt  — target hover altitude in metres (default: 3.0)
  goal_x     — ENU X of the navigation target in metres (default: 8.0)
  goal_y     — ENU Y of the navigation target in metres (default: 8.0)
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_bringup')

    world_file  = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    model_path  = os.path.join(pkg_dir, 'models')
    urdf_file   = os.path.join(pkg_dir, 'models', 'x500_vision_lidar', 'model.urdf.xacro')
    bridge_cfg  = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'multi_drone.rviz')

    _px4_dir = os.environ.get('PX4_DIR',
                              '/home/telemaque/px4_workspace/PX4-Autopilot')
    _px4_models = os.path.join(_px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_resource_path = (
        f"{model_path}:{_px4_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    XRCE_AGENT = os.environ.get(
        'XRCE_AGENT',
        '/home/telemaque/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')
    px4_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'launch_px4_instance.sh')
    heartbeat_script = os.path.join(
        get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', 'gcs_heartbeat.py')

    # ── Launch args ──────────────────────────────────────────────────────────
    use_rviz_arg  = DeclareLaunchArgument('use_rviz',  default_value='false',
                                          description='Launch RViz2')
    hover_alt_arg = DeclareLaunchArgument('hover_alt', default_value='3.0',
                                          description='Hover altitude in metres')
    goal_x_arg    = DeclareLaunchArgument('goal_x', default_value='8.0',
                                          description='Navigation goal ENU X (metres)')
    goal_y_arg    = DeclareLaunchArgument('goal_y', default_value='8.0',
                                          description='Navigation goal ENU Y (metres)')

    # ── Gazebo (server-only for stability) ───────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v4', '-s', '--headless-rendering', world_file],
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
            'GZ_IP': '127.0.0.1',
            'QT_QPA_PLATFORM': 'offscreen',
            'DISPLAY': '',
        },
        output='screen',
    )

    # ── Drone spawning (world name is "empty") ────────────────────────────────
    model_sdf = os.path.join(model_path, 'x500_vision_lidar', 'model.sdf')

    drone1_spawn = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', (
                f'sdf_filename: "{model_sdf}", '
                'name: "x500_d1", '
                'pose: {position: {x: 0.0, y: -2.0, z: 0.5}}'
            ),
        ],
        output='screen',
    )

    drone2_spawn = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', (
                f'sdf_filename: "{model_sdf}", '
                'name: "x500_d2", '
                'pose: {position: {x: 0.0, y: 2.0, z: 0.5}}'
            ),
        ],
        output='screen',
    )

    # ── ROS-Gazebo bridge ─────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_cfg}],
        ros_arguments=['--log-level', 'warn'],
        additional_env={
            'GZ_LOG_LEVEL': 'warn',
            'GZ_IP': '127.0.0.1',
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        },
        output='screen',
    )

    # ── Robot state publishers (TF tree) ─────────────────────────────────────
    def make_rsp(drone_ns):
        urdf_content = subprocess.check_output(
            ['xacro', urdf_file, f'ns:={drone_ns}']
        ).decode('utf-8')
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=drone_ns,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_content,
                'frame_prefix': f'{drone_ns}/',
            }],
            output='screen',
        )

    rsp_d1 = make_rsp('d1')
    rsp_d2 = make_rsp('d2')

    # ── PX4 SITL + MicroXRCE agents ───────────────────────────────────────────
    xrce_d1 = ExecuteProcess(
        cmd=[XRCE_AGENT, 'udp4', '-p', '8888'],
        name='xrce_agent_d1', output='screen')
    xrce_d2 = ExecuteProcess(
        cmd=[XRCE_AGENT, 'udp4', '-p', '8889'],
        name='xrce_agent_d2', output='screen')

    px4_d1 = ExecuteProcess(
        cmd=['bash', px4_script, '0', 'x500_d1', 'd1'],
        name='px4_d1',
        additional_env={'PX4_DIR': _px4_dir, 'PX4_ENABLE_VIO': '0'},
        output='screen',
    )
    px4_d2 = ExecuteProcess(
        cmd=['bash', px4_script, '1', 'x500_d2', 'd2'],
        name='px4_d2',
        additional_env={'PX4_DIR': _px4_dir, 'PX4_ENABLE_VIO': '0'},
        output='screen',
    )

    gcs_heartbeat = ExecuteProcess(
        cmd=['python3', heartbeat_script, '2'],
        name='gcs_heartbeat', output='screen')

    # ── Offboard controllers ──────────────────────────────────────────────────
    ctrl_d1 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d1',
        parameters=[{'drone_ns': 'd1', 'hover_alt': LaunchConfiguration('hover_alt')}],
        output='screen',
    )
    ctrl_d2 = Node(
        package='px4_offboard',
        executable='offboard_controller',
        name='offboard_controller_d2',
        parameters=[{'drone_ns': 'd2', 'hover_alt': LaunchConfiguration('hover_alt')}],
        output='screen',
    )

    # ── Navigation goal publisher ──────────────────────────────────────────────
    # A single Python node publishes goals for both drones from ONE DDS
    # participant. Using two separate `ros2 topic pub` processes creates two
    # simultaneous DDS discovery events which spike CPU and cause XRCE-DDS
    # timesync jumps → attitude health failures on PX4.
    # Goals are in the ENU map frame (geometry_msgs/Point).
    # The controller ignores Point.z and uses hover_alt for altitude.
    # d2 gets goal_y + 4 m to maintain the 4 m lateral separation from spawn.
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_y_d2 = PythonExpression(["str(float('", goal_y, "') + 4.0)"])

    goal_publisher = ExecuteProcess(
        cmd=[
            'python3', '-c',
            [
                'import rclpy, time\n'
                'from rclpy.node import Node\n'
                'from geometry_msgs.msg import Point\n'
                'rclpy.init()\n'
                'n = Node("goal_injector")\n'
                'p1 = n.create_publisher(Point, "/d1/goal_pose", 10)\n'
                'p2 = n.create_publisher(Point, "/d2/goal_pose", 10)\n'
                'time.sleep(1.0)\n'          # wait for DDS discovery to settle
                'm1 = Point(); m1.x = ', goal_x, '; m1.y = ', goal_y,   '; m1.z = 0.0\n'
                'm2 = Point(); m2.x = ', goal_x, '; m2.y = ', goal_y_d2, '; m2.z = 0.0\n'
                'p1.publish(m1)\n'
                'time.sleep(2.0)\n'          # stagger d2 by 2 s — avoids overlapping manoeuvres
                'p2.publish(m2)\n'
                'time.sleep(0.5)\n'
                'rclpy.shutdown()\n',
            ],
        ],
        name='goal_injector',
        output='screen',
    )

    # ── RViz (optional) ───────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        use_rviz_arg,
        hover_alt_arg,
        goal_x_arg,
        goal_y_arg,

        # Restrict DDS to loopback — eliminates multicast jitter
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>'),

        # t=0  — Gazebo server
        gazebo,

        # t=5  — spawn d1; t=7 spawn d2; t=8 bridge + TF + RViz
        TimerAction(period=5.0,  actions=[drone1_spawn]),
        TimerAction(period=7.0,  actions=[drone2_spawn]),
        TimerAction(period=8.0,  actions=[bridge, rsp_d1, rsp_d2, rviz]),

        # t=10 — PX4 SITL + XRCE agents + GCS heartbeat
        #        d2 PX4 staggered by 5 s to avoid symlink race in build dir
        TimerAction(period=10.0, actions=[xrce_d1, xrce_d2, px4_d1, gcs_heartbeat]),
        TimerAction(period=15.0, actions=[px4_d2]),

        # t=30 — offboard controllers (GPS EKF2 fully converged by now)
        TimerAction(period=30.0, actions=[ctrl_d1, ctrl_d2]),

        # t=50 — send navigation goals (both drones stable in HOVER by now)
        TimerAction(period=50.0, actions=[goal_publisher]),
    ])
