#!/usr/bin/env python3
"""
Lidar enricher for LIO-SAM:
- Adds 'ring' (uint16) if missing, derived from vertical angle.
- Adds 'time' (float32) per point using horizontal index and scan rate.
Default parameters suit VLP-16 style sensors (16 rings, 360° / 1800 cols, 10 Hz).
"""
import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


SIZE_OF = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


class LidarEnricher(Node):
    def __init__(self):
        super().__init__('lidar_enricher')

        self.declare_parameter('n_scan', 16)
        self.declare_parameter('horizon_scan', 1800)
        self.declare_parameter('scan_rate', 10.0)         # Hz
        self.declare_parameter('min_vert_deg', -15.0)     # VLP-16 default
        self.declare_parameter('max_vert_deg', 15.0)
        self.declare_parameter('input_topic', 'points_raw')
        self.declare_parameter('output_topic', 'points_enriched')

        qos = rclpy.qos.QoSProfile(
            depth=5,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, output_topic, qos)

        self.get_logger().info(
            f"Enriching {input_topic} → {output_topic} "
            f"(n_scan={self.n_scan}, horizon={self.horizon_scan}, rate={self.scan_rate} Hz)"
        )

    @property
    def n_scan(self):
        return int(self.get_parameter('n_scan').value)

    @property
    def horizon_scan(self):
        return int(self.get_parameter('horizon_scan').value)

    @property
    def scan_rate(self):
        return float(self.get_parameter('scan_rate').value)

    @property
    def min_vert_rad(self):
        return math.radians(float(self.get_parameter('min_vert_deg').value))

    @property
    def max_vert_rad(self):
        return math.radians(float(self.get_parameter('max_vert_deg').value))

    def cb(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        has_ring = 'ring' in field_names
        has_time = 'time' in field_names

        points_iter = pc2.read_points(msg, skip_nans=False, field_names=field_names)
        points_list = list(points_iter)
        if not points_list:
            return

        # Build output fields
        fields_out: List[PointField] = list(msg.fields)
        offset = 0
        if fields_out:
            last = fields_out[-1]
            offset = last.offset + SIZE_OF[last.datatype] * last.count

        if not has_ring:
            fields_out.append(PointField(name='ring', offset=offset,
                                         datatype=PointField.UINT16, count=1))
            offset += 2
        if not has_time:
            fields_out.append(PointField(name='time', offset=offset,
                                         datatype=PointField.FLOAT32, count=1))

        enriched_points = []
        vert_res = (self.max_vert_rad - self.min_vert_rad) / max(self.n_scan - 1, 1)

        for idx, pt in enumerate(points_list):
            pt_list = list(pt)
            # Extend for missing fields to keep indexing simple
            if not has_ring:
                x, y, z = pt_list[field_names.index('x')], pt_list[field_names.index('y')], pt_list[field_names.index('z')]
                r = math.sqrt(x * x + y * y + z * z) or 1e-6
                vert_angle = math.asin(z / r)
                ring = int(round((vert_angle - self.min_vert_rad) / vert_res))
                ring = max(0, min(self.n_scan - 1, ring))
                pt_list.append(ring)
            if not has_time:
                horiz_idx = idx % self.horizon_scan
                time = horiz_idx / (self.horizon_scan * self.scan_rate)
                pt_list.append(time)
            enriched_points.append(tuple(pt_list))

        out_msg = pc2.create_cloud(msg.header, fields_out, enriched_points)
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = LidarEnricher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
