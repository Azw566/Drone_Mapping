#!/usr/bin/env python3
"""
Broadcast map->base_link transforms from Gazebo pose topics.

This is the minimal TF needed for OctoMap when we skip PX4 and LIO-SAM and map
directly from bridged Gazebo lidar clouds.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class GazeboPoseToTF(Node):
    def __init__(self):
        super().__init__('gz_pose_to_tf')

        self.declare_parameter('pose_topic', '/d1/pose')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('parent_frame', 'd1/map')
        self.declare_parameter('child_frame', 'd1/base_link')

        self._parent_frame = str(self.get_parameter('parent_frame').value)
        self._child_frame = str(self.get_parameter('child_frame').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        self._broadcaster = TransformBroadcaster(self)
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, pose_qos)
        else:
            self.create_subscription(PoseStamped, pose_topic, self._pose_cb, pose_qos)

        self.get_logger().info(
            f'Broadcasting TF {self._parent_frame} -> {self._child_frame} '
            f'from {odom_topic or pose_topic}')

    def _pose_cb(self, msg: PoseStamped):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self._parent_frame
        tf_msg.child_frame_id = self._child_frame
        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation
        self._broadcaster.sendTransform(tf_msg)

    def _odom_cb(self, msg: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self._parent_frame
        tf_msg.child_frame_id = self._child_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = GazeboPoseToTF()
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
