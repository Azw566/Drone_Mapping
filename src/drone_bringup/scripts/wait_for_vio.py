#!/usr/bin/env python3
"""
Wait for VehicleOdometry messages on a PX4 VIO input topic.

Exit status:
  0 if at least COUNT messages are received before TIMEOUT
  1 otherwise
"""

import argparse
import sys
import time

import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class WaitForVio(Node):
    def __init__(self, topic: str, target_count: int):
        super().__init__('wait_for_vio')
        self._target_count = target_count
        self._count = 0
        self.create_subscription(
            VehicleOdometry,
            topic,
            self._cb,
            PX4_QOS,
        )

    @property
    def done(self) -> bool:
        return self._count >= self._target_count

    def _cb(self, _msg: VehicleOdometry) -> None:
        self._count += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('topic')
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = WaitForVio(args.topic, args.count)

    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if node.done else 1


if __name__ == '__main__':
    sys.exit(main())
