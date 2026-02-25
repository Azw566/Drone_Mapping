"""Quick reference
- Gazebo + world
- Spawn d1 at entrance (-7, 0, 0.5)
- Spawn d2 at entrance offset (-7, 2, 0.5)
- Start bridges + TF publishers
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_bringup')

    world_file = os.path.join(pkg_dir, 'worlds', 'maze.sdf')
    model_path = os.path.join(pkg_dir, 'models')
    urdf_file = os.path.join(pkg_dir, 'models', 'x500_vision_lidar', 'model.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'multi_drone.rviz')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{model_path}:{gz_resource_path}"


    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', '-v4', world_file
        ],
        output='screen',
    )

    drone1_spawn = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/maze/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', (
                f'sdf_filename: "{os.path.join(model_path, "x500_vision_lidar", "model.sdf")}", '
                'name: "x500_d1", '
                'pose: {position: {x: -1.0, y: -8.0, z: 0.5}}'
            ),
        ],
        output='screen',
    )

    # Delay drone 2 spawn by 2 seconds to avoid Gazebo race conditions.
    # If both spawn simultaneously, Gazebo sometimes assigns duplicate IDs.
    drone2_spawn = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/maze/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', (
                        f'sdf_filename: "{os.path.join(model_path, "x500_vision_lidar", "model.sdf")}", '
                        'name: "x500_d2", '
                        'pose: {position: {x: 1.0, y: -8.0, z: 0.5}}'
                    ),
                ],
                output='screen',
            )
        ],
    )


    # Single bridge configured via YAML to avoid magnetometer / barometer topics.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )


    def create_robot_state_publisher(drone_ns):
        """Creates robot_state_publisher for one drone."""
        # Process xacro to get URDF string
        import subprocess
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
                # frame_prefix adds the namespace to all TF frame names.
                # Without it, both drones would publish "base_link" and
                # TF would be confused. With it:
                #   drone 1: d1/base_link, d1/lidar_link, d1/camera_link
                #   drone 2: d2/base_link, d2/lidar_link, d2/camera_link
                'frame_prefix': f'{drone_ns}/',
            }],
            output='screen',
        )

    rsp_d1 = create_robot_state_publisher('d1')
    rsp_d2 = create_robot_state_publisher('d2')


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
        gazebo,
        # Wait for Gazebo to initialize before spawning
        TimerAction(period=5.0, actions=[drone1_spawn]),
        TimerAction(period=7.0, actions=[drone2_spawn]),
        # Wait for models to spawn before starting bridge
        TimerAction(period=8.0, actions=[bridge]),
        TimerAction(period=8.0, actions=[rsp_d1, rsp_d2]),
        rviz,
    ])
