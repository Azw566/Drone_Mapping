#!/usr/bin/env python3
"""
Transform bridged Gazebo lidar clouds into the drone map/world frame.

This avoids TF timing issues for the simple mapping path by publishing clouds
already expressed in the fixed map frame expected by OctoMap.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)),
        (2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)),
    )


class CloudToMap(Node):
    def __init__(self):
        super().__init__('cloud_to_map')

        self.declare_parameter('input_topic', '/d1/points_raw')
        self.declare_parameter('pose_topic', '/d1/pose')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('output_topic', '/d1/points_mapped')
        self.declare_parameter('output_frame_id', 'd1/map')
        self.declare_parameter('sensor_offset_xyz', [0.0, 0.0, 0.12])
        self.declare_parameter('min_range_m', 0.8)
        self.declare_parameter('max_range_m', 10.5)
        self.declare_parameter('min_world_z_m', 0.35)
        self.declare_parameter('max_world_z_m', 2.25)
        self.declare_parameter('world_x_bounds_m', [-10.4, 10.4])
        self.declare_parameter('world_y_bounds_m', [-10.4, 10.4])
        self.declare_parameter('settle_after_jump_s', 1.5)
        self.declare_parameter('jump_distance_m', 0.35)

        self._output_frame = str(self.get_parameter('output_frame_id').value)
        self._sensor_offset = list(self.get_parameter('sensor_offset_xyz').value)
        self._min_range_m = float(self.get_parameter('min_range_m').value)
        self._max_range_m = float(self.get_parameter('max_range_m').value)
        self._min_world_z_m = float(self.get_parameter('min_world_z_m').value)
        self._max_world_z_m = float(self.get_parameter('max_world_z_m').value)
        self._world_x_bounds = list(self.get_parameter('world_x_bounds_m').value)
        self._world_y_bounds = list(self.get_parameter('world_y_bounds_m').value)
        self._settle_after_jump_s = float(self.get_parameter('settle_after_jump_s').value)
        self._jump_distance_m = float(self.get_parameter('jump_distance_m').value)
        self._pose = None
        self._last_pose_xyz = None
        self._suppress_until_ns = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        input_topic = str(self.get_parameter('input_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        else:
            self.create_subscription(PoseStamped, pose_topic, self._pose_cb, qos)
        self.create_subscription(PointCloud2, input_topic, self._cloud_cb, qos)
        self._pub = self.create_publisher(PointCloud2, output_topic, qos)

        self.get_logger().info(
            f'Transforming {input_topic} + {odom_topic or pose_topic} '
            f'-> {output_topic} in {self._output_frame}')

    def _pose_cb(self, msg: PoseStamped):
        self._pose = msg.pose

    def _odom_cb(self, msg: Odometry):
        self._pose = msg.pose.pose
        self._update_jump_guard(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def _update_jump_guard(self, x: float, y: float, z: float):
        if self._last_pose_xyz is not None:
            dx = x - self._last_pose_xyz[0]
            dy = y - self._last_pose_xyz[1]
            dz = z - self._last_pose_xyz[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) > self._jump_distance_m:
                self._suppress_until_ns = (
                    self.get_clock().now().nanoseconds +
                    int(self._settle_after_jump_s * 1e9)
                )
        self._last_pose_xyz = (x, y, z)

    def _cloud_cb(self, msg: PointCloud2):
        if self._pose is None:
            return
        if self.get_clock().now().nanoseconds < self._suppress_until_ns:
            return

        q = self._pose.orientation
        pose_values = (
            self._pose.position.x,
            self._pose.position.y,
            self._pose.position.z,
            q.x, q.y, q.z, q.w,
        )
        if not all(float('-inf') < float(v) < float('inf') for v in pose_values):
            return

        rot = _quat_to_rot(q.x, q.y, q.z, q.w)
        px = self._pose.position.x
        py = self._pose.position.y
        pz = self._pose.position.z
        ox, oy, oz = self._sensor_offset

        ox_w = rot[0][0] * ox + rot[0][1] * oy + rot[0][2] * oz
        oy_w = rot[1][0] * ox + rot[1][1] * oy + rot[1][2] * oz
        oz_w = rot[2][0] * ox + rot[2][1] * oy + rot[2][2] * oz

        mapped = []
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if not all(float('-inf') < float(v) < float('inf') for v in (x, y, z)):
                continue
            point_range = math.sqrt(x * x + y * y + z * z)
            if point_range < self._min_range_m or point_range > self._max_range_m:
                continue

            wx = px + ox_w + rot[0][0] * x + rot[0][1] * y + rot[0][2] * z
            wy = py + oy_w + rot[1][0] * x + rot[1][1] * y + rot[1][2] * z
            wz = pz + oz_w + rot[2][0] * x + rot[2][1] * y + rot[2][2] * z

            if wz < self._min_world_z_m or wz > self._max_world_z_m:
                continue
            if wx < self._world_x_bounds[0] or wx > self._world_x_bounds[1]:
                continue
            if wy < self._world_y_bounds[0] or wy > self._world_y_bounds[1]:
                continue

            mapped.append((wx, wy, wz))

        if not mapped:
            return

        out = pc2.create_cloud_xyz32(msg.header, mapped)
        out.header.frame_id = self._output_frame
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CloudToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
