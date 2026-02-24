#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


def has_field(msg: PointCloud2, name: str) -> bool:
    return any(f.name == name for f in msg.fields)


class PointCloudAdapter(Node):
    """
    Adds fields required by LIO-SAM for Velodyne-style clouds:
    - ring (uint16): laser channel index [0..N_SCAN-1]
    - time (float32): relative time within scan (seconds), approx (if missing)
    """
    def __init__(self):
        super().__init__('pointcloud_adapter')

        self.declare_parameter('input_topic', 'points_raw')
        self.declare_parameter('output_topic', 'points_adapted')
        self.declare_parameter('n_scan', 16)               # VLP-16
        self.declare_parameter('frame_id', '')             # optional override
        self.declare_parameter('use_best_effort', True)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.n_scan = int(self.get_parameter('n_scan').value)
        self.override_frame = self.get_parameter('frame_id').value

        use_be = bool(self.get_parameter('use_best_effort').value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if use_be else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, qos)

        self.get_logger().info(f"pointcloud_adapter: {in_topic} -> {out_topic} (N_SCAN={self.n_scan})")

    def cb(self, msg: PointCloud2):
        need_ring = not has_field(msg, 'ring')
        need_time = not has_field(msg, 'time')

        if not need_ring and not need_time and not self.override_frame:
            self.pub.publish(msg)
            return

        field_names = [f.name for f in msg.fields]
        read_fields = []
        for fn in ['x', 'y', 'z', 'intensity']:
            if fn in field_names:
                read_fields.append(fn)

        if len(read_fields) < 3:
            self.get_logger().warn("Incoming cloud missing x/y/z; skipping.")
            return

        points = list(pc2.read_points(msg, field_names=read_fields, skip_nans=True))
        if not points:
            return

        out_fields = []
        offset = 0
        for fn in read_fields:
            out_fields.append(PointField(name=fn, offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4

        if need_ring:
            out_fields.append(PointField(name='ring', offset=offset, datatype=PointField.UINT16, count=1))
            offset += 2
            if offset % 4 != 0:
                offset += (4 - (offset % 4))

        if need_time:
            out_fields.append(PointField(name='time', offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4

        def ring_from_xyz(x, y, z):
            r = math.sqrt(x * x + y * y)
            if r < 1e-6:
                return 0
            ang = math.degrees(math.atan2(z, r))  # vertical angle
            ring = int(round((ang + 15.0) * (self.n_scan - 1) / 30.0))
            return max(0, min(self.n_scan - 1, ring))

        out_points = []
        for p in points:
            data = {read_fields[i]: float(p[i]) for i in range(len(read_fields))}
            row = [data[fn] for fn in read_fields]
            if need_ring:
                row.append(ring_from_xyz(data['x'], data['y'], data['z']))
            if need_time:
                row.append(0.0)
            out_points.append(tuple(row))

        out_msg = pc2.create_cloud(msg.header, out_fields, out_points)

        if self.override_frame:
            out_msg.header.frame_id = self.override_frame

        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = PointCloudAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
