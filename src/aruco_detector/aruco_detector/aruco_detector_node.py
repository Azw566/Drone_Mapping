"""
aruco_detector_node.py

Detects ArUco markers in the drone's RGB camera feed and publishes their
6-DoF world pose as drone_interfaces/ArucoDetection messages.

Pipeline:
  1. Receive sensor_msgs/Image + sensor_msgs/CameraInfo
  2. Detect ArUco corners with OpenCV (handles both ≥4.7 and <4.7 API)
  3. Solve PnP → marker pose in camera_optical_frame
  4. Transform to map frame via TF2 (camera_optical_frame is in the TF tree,
     published by robot_state_publisher from the URDF)
  5. Publish drone_interfaces/ArucoDetection to /{drone_ns}/aruco/detections

The 'detected_by' and topic structure are ready for multi-drone POI merging
but the RegisterPOI service call is intentionally left for future integration.
"""

import numpy as np
import cv2
import rclpy
import rclpy.duration
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # registers PoseStamped with tf2 do_transform

from drone_interfaces.msg import ArucoDetection


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('image_topic',       '/d1/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/d1/camera/camera_info')
        self.declare_parameter('output_topic',      '/d1/aruco/detections')
        self.declare_parameter('drone_id',          'd1')
        self.declare_parameter('map_frame',         'd1/map')
        # camera_optical_frame is published by robot_state_publisher (URDF):
        #   base_link → camera_link → camera_optical_frame
        # OpenCV convention: Z=forward, X=right, Y=down  ← matches PnP output
        self.declare_parameter('camera_optical_frame', 'd1/camera_optical_frame')
        self.declare_parameter('marker_size',       0.2)   # physical side length [m]
        self.declare_parameter('aruco_dict_id',     0)     # 0 = DICT_4X4_50

        image_topic      = self.get_parameter('image_topic').value
        info_topic       = self.get_parameter('camera_info_topic').value
        output_topic     = self.get_parameter('output_topic').value
        self.drone_id    = self.get_parameter('drone_id').value
        self.map_frame   = self.get_parameter('map_frame').value
        self.cam_frame   = self.get_parameter('camera_optical_frame').value
        self.marker_size = self.get_parameter('marker_size').value
        dict_id          = self.get_parameter('aruco_dict_id').value

        # ── ArUco detector (API ≥4.7 / <4.7 compat) ─────────────────────
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        try:
            det_params         = cv2.aruco.DetectorParameters()
            self._detector     = cv2.aruco.ArucoDetector(aruco_dict, det_params)
            self._new_aruco    = True
        except AttributeError:
            det_params         = cv2.aruco.DetectorParameters_create()
            self._aruco_dict   = aruco_dict
            self._det_params   = det_params
            self._new_aruco    = False

        # 3-D object points of the marker corners in marker frame:
        # ArUco corner order: TL, TR, BR, BL (looking at marker face)
        # Marker Y-up, X-right, Z toward camera.
        h = self.marker_size / 2.0
        self._obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

        # ── Camera intrinsics (filled from CameraInfo) ────────────────────
        self._K = None
        self._D = None
        self._bridge = CvBridge()

        # ── TF2 ──────────────────────────────────────────────────────────
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── ROS interfaces ────────────────────────────────────────────────
        self.create_subscription(CameraInfo, info_topic, self._info_cb,  1)
        self.create_subscription(Image,      image_topic, self._image_cb, 5)
        self._pub = self.create_publisher(ArucoDetection, output_topic, 10)

        self.get_logger().info(
            f'ArUco detector ready | drone={self.drone_id} '
            f'| image={image_topic} | output={output_topic}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        if self._K is None:
            self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def _image_cb(self, msg: Image):
        if self._K is None:
            return  # wait for intrinsics

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if self._new_aruco:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._det_params)

        if ids is None or len(ids) == 0:
            return

        stamp = msg.header.stamp

        for i, marker_id in enumerate(ids.flatten()):
            img_pts = corners[i].reshape(4, 2).astype(np.float32)

            # PnP: pose of marker in camera_optical_frame
            ret, rvec, tvec = cv2.solvePnP(
                self._obj_pts, img_pts,
                self._K, self._D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ret:
                continue

            # Build PoseStamped in camera_optical_frame
            pose_cam = PoseStamped()
            pose_cam.header.stamp    = stamp
            pose_cam.header.frame_id = self.cam_frame

            tvec = tvec.flatten()
            pose_cam.pose.position.x = float(tvec[0])
            pose_cam.pose.position.y = float(tvec[1])
            pose_cam.pose.position.z = float(tvec[2])

            rot_mat, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = self._rot_to_quat(rot_mat)
            pose_cam.pose.orientation.x = qx
            pose_cam.pose.orientation.y = qy
            pose_cam.pose.orientation.z = qz
            pose_cam.pose.orientation.w = qw

            # Transform to map frame
            try:
                pose_world = self._tf_buf.transform(
                    pose_cam,
                    self.map_frame,
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except TransformException as exc:
                self.get_logger().debug(f'TF {self.cam_frame}→{self.map_frame}: {exc}')
                continue

            # Publish
            det = ArucoDetection()
            det.tag_id      = int(marker_id)
            det.world_pose  = pose_world
            det.confidence  = self._confidence(corners[i])
            det.detected_by = self.drone_id
            self._pub.publish(det)

    # ── Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _rot_to_quat(R):
        """Convert 3×3 rotation matrix to quaternion (x, y, z, w)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return x, y, z, w

    @staticmethod
    def _confidence(corners):
        """Marker pixel area → confidence ∈ [0, 1]. 10 000 px² → 1.0."""
        pts  = corners.reshape(4, 2).astype(np.float32)
        area = float(cv2.contourArea(pts))
        return min(1.0, area / 10_000.0)


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
