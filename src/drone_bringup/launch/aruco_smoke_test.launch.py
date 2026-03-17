import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_bringup')

    world_file = os.path.join(pkg_dir, 'worlds', 'aruco_smoke.sdf')
    model_path = os.path.join(pkg_dir, 'models')
    urdf_file = os.path.join(pkg_dir, 'models', 'x500_vision_lidar', 'model.urdf.xacro')
    bridge_cfg = os.path.join(pkg_dir, 'config', 'bridge.yaml')

    px4_dir = os.environ.get('PX4_DIR', '/home/telemaque/px4_workspace/PX4-Autopilot')
    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_resource_path = f"{model_path}:{px4_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"

    use_rviz = LaunchConfiguration('use_rviz')

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

    drone_spawn = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/maze/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', (
                f'sdf_filename: "{os.path.join(model_path, "x500_vision_lidar", "model.sdf")}", '
                'name: "x500_d1", '
                'pose: {position: {x: -3.65, y: 0.0, z: 3.0}}'
            ),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='aruco_smoke_bridge',
        parameters=[{'config_file': bridge_cfg}],
        ros_arguments=['--log-level', 'warn'],
        additional_env={
            'GZ_LOG_LEVEL': 'warn',
            'GZ_IP': '127.0.0.1',
            'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        },
        output='screen',
    )

    urdf_content = subprocess.check_output(['xacro', urdf_file, 'ns:=d1']).decode('utf-8')
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='d1',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content,
            'frame_prefix': 'd1/',
        }],
        output='screen',
    )

    map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link',
        arguments=[
            '--x', '-3.65',
            '--y', '0.0',
            '--z', '3.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'd1/map',
            '--child-frame-id', 'd1/base_link',
        ],
        output='screen',
    )

    aruco = Node(
        package='aruco_detector',
        executable='aruco_detector',
        name='aruco_detector_d1',
        output='screen',
        parameters=[{
            'image_topic': '/d1/camera/image_raw',
            'camera_info_topic': '/d1/camera/camera_info',
            'output_topic': '/d1/aruco/detections',
            'drone_id': 'd1',
            'map_frame': 'd1/map',
            'camera_optical_frame': 'd1/camera_optical_frame',
            'marker_size': 0.6,
            'aruco_dict_id': 0,
            'min_confidence': 0.01,
            'max_detection_distance_m': 10.0,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General>'
            '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
            '</General></Domain></CycloneDDS>',
        ),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        gazebo,
        TimerAction(period=3.0, actions=[drone_spawn]),
        TimerAction(period=5.0, actions=[bridge, rsp, map_to_base]),
        TimerAction(period=7.0, actions=[aruco]),
        rviz,
    ])
