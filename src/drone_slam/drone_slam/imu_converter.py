#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu


class ImuConverter(Node):
    """
    Enforce frame_id and topic naming for LIO-SAM.
    """
    def __init__(self):
        super().__init__('imu_converter')

        self.declare_parameter('input_topic', 'imu/data')
        self.declare_parameter('output_topic', 'imu/data_adapted')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_best_effort', True)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').value

        use_be = bool(self.get_parameter('use_best_effort').value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if use_be else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.sub = self.create_subscription(Imu, in_topic, self.cb, qos)

        self.get_logger().info(f"imu_converter: {in_topic} -> {out_topic} (frame_id={self.frame_id})")

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
