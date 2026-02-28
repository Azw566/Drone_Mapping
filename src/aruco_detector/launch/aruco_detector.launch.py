"""
aruco_detector.launch.py

Launches one ArUco detector node for a single drone.
Pass drone_ns:=d2 to spawn a second instance for drone 2.

Topic layout (default drone_ns=d1):
  subscribes:  /d1/camera/image_raw
               /d1/camera/camera_info
  publishes:   /d1/aruco/detections   (drone_interfaces/ArucoDetection)

TF used:
  d1/camera_optical_frame → d1/map   (via robot_state_publisher + LIO-SAM)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_ns = LaunchConfiguration('drone_ns')

    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_ns',
            default_value='d1',
            description='Drone namespace (d1 or d2)'),
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.2',
            description='Physical ArUco marker side length in metres'),
        DeclareLaunchArgument(
            'aruco_dict_id',
            default_value='0',
            description='OpenCV ArUco dictionary ID (0=DICT_4X4_50)'),

        Node(
            package='aruco_detector',
            executable='aruco_detector',
            name=['aruco_detector_', drone_ns],
            output='screen',
            parameters=[{
                'image_topic':           ['/', drone_ns, '/camera/image_raw'],
                'camera_info_topic':     ['/', drone_ns, '/camera/camera_info'],
                'output_topic':          ['/', drone_ns, '/aruco/detections'],
                'drone_id':              drone_ns,
                'map_frame':             [drone_ns, '/map'],
                'camera_optical_frame':  [drone_ns, '/camera_optical_frame'],
                'marker_size':           LaunchConfiguration('marker_size'),
                'aruco_dict_id':         LaunchConfiguration('aruco_dict_id'),
            }],
        ),
    ])
